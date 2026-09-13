#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>

class MyAhrsDriverNode : public rclcpp::Node
{
public:
    MyAhrsDriverNode() : Node("myahrs_driver_node"), serial_fd_(-1)
    {
        // 파라미터 선언 (기본값 설정)
        this->declare_parameter<std::string>("port", "/dev/ttyACM1");
        this->declare_parameter<int>("baudrate", 460800); // 가이드 기준 고속 보드레이트
        this->declare_parameter<std::string>("frame_id", "imu_link");
        // [2026-08-23 신규] myAHRS+ continuous output divider -- 실측 기본
        // 10Hz(EKF sensor_timeout 마진 부족 -> 발산 진단 세션 참고)를 협의
        // 프로토콜(@divider,<N>, output_hz = 100/divider)로 직접 올리기
        // 위함. 기본값 10 = 기존 동작(10Hz)과 100% 호환 -- 이 파라미터를
        // 명시적으로 안 넘기면 이전과 똑같이 동작한다.
        this->declare_parameter<int>("output_divider", 10);

        // orientation covariance 대각 성분 (정지 상태 실측 기반, 2000 샘플, imu_covariance_calibrator.py 사용)
        this->declare_parameter<double>("orientation_covariance_roll", 0.00000594);
        this->declare_parameter<double>("orientation_covariance_pitch", 0.00003487);
        this->declare_parameter<double>("orientation_covariance_yaw", 0.00051956);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        int output_divider = this->get_parameter("output_divider").as_int();

        orientation_covariance_roll_ = this->get_parameter("orientation_covariance_roll").as_double();
        orientation_covariance_pitch_ = this->get_parameter("orientation_covariance_pitch").as_double();
        orientation_covariance_yaw_ = this->get_parameter("orientation_covariance_yaw").as_double();

        // [2026-08-23] 프로토콜이 실제로 지원한다고 확인된 divider만 허용
        // (사용자 제공 사양 기준 -- 이 레포엔 myAHRS+ 공식 프로토콜 문서가
        // 없어서 코드 근거로는 재검증 불가, 매뉴얼 기준을 그대로 신뢰함).
        // 임의 반올림/보정 금지 -- 지원 안 되는 값이면 그냥 에러 내고 종료.
        static const std::map<int, int> kDividerToHz = {
            {1, 100}, {2, 50}, {4, 25}, {5, 20}, {10, 10},
        };
        auto it = kDividerToHz.find(output_divider);
        if (it == kDividerToHz.end()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported output_divider=%d (지원값: 1,2,4,5,10 -> 100,50,25,20,10 Hz). "
                "임의로 반올림하지 않고 노드를 시작하지 않습니다.",
                output_divider);
            return;
        }
        const int expected_hz = it->second;

        // IMU 퍼블리셔 등록
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

        // 시리얼 포트 오픈
        if (!initSerial(port, baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Success to open myAHRS+ on %s (%d bps)", port.c_str(), baudrate);

        // [2026-08-23 신규] read loop 시작 전에 divider 설정 명령 전송.
        // ⚠️ 이 레포에 myAHRS+ 공식 프로토콜 문서가 없어서 wire format(개행
        // 문자, ACK 유무)을 코드로 재검증하지 못했음 -- 사용자 제공 사양을
        // 그대로 신뢰해서 구현. ACK를 기다리거나 검증하지 않으므로, 실제로
        // 적용됐는지는 반드시 `ros2 topic hz /imu`로 런타임에 확인할 것.
        if (!sendDividerCommand(output_divider)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to send @divider command to myAHRS+ -- output rate may remain at "
                "device's current setting.");
        }
        RCLCPP_INFO(
            this->get_logger(), "[myAHRS] output divider = %d", output_divider);
        RCLCPP_INFO(
            this->get_logger(), "[myAHRS] expected output rate = %d Hz (ACK 없음, "
            "실제 반영 여부는 'ros2 topic hz /imu'로 확인할 것)", expected_hz);

        // 데이터 수신을 위한 타이머 스레드 (100Hz 루프)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&MyAhrsDriverNode::readSerialData, this));
    }

    ~MyAhrsDriverNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool initSerial(const std::string& port, int baudrate)
    {
        serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        struct termios toptions;
        tcgetattr(serial_fd_, &toptions);

        speed_t brate = B460800;
        if (baudrate == 115200) brate = B115200;

        cfsetispeed(&toptions, brate);
        cfsetospeed(&toptions, brate);

        toptions.c_cflag &= ~PARENB; // No parity
        toptions.c_cflag &= ~CSTOPB; // 1 stop bit
        toptions.c_cflag &= ~CSIZE;
        toptions.c_cflag |= CS8;     // 8 bits
        toptions.c_cflag |= CREAD | CLOCAL;

        toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        toptions.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INPCK | ISTRIP);
        toptions.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &toptions);
        return true;
    }

    // [2026-08-23 신규] myAHRS+ continuous output divider 설정.
    // ⚠️ 사용자 제공 사양 기준 구현(이 레포엔 공식 프로토콜 문서 없음,
    // 클래스 상단 docstring 참고). 개행은 "\r\n"으로 보냄 -- 이 장비가
    // 정확히 어떤 개행을 요구하는지 문서로 확인 못 했지만, 기존
    // readSerialData()가 장비 쪽에서 오는 프레임의 trailing '\r'을 이미
    // 벗겨내는 걸 보면 장비 자신도 CRLF 계열을 쓰는 것으로 보여 이를 따름.
    // ACK 응답을 기다리거나 검증하지 않는다 -- 응답 유무/형식이 문서로
    // 확인 안 됐으므로, 여기서 잘못 파싱해서 오히려 문제를 만들지 않기
    // 위해 "보내기만 하고 결과는 런타임 rate 실측으로 확인"하는 쪽을 택함.
    bool sendDividerCommand(int divider)
    {
        const std::string cmd = "@divider," + std::to_string(divider) + "\r\n";
        const ssize_t written = write(serial_fd_, cmd.data(), cmd.size());
        if (written < 0 || static_cast<size_t>(written) != cmd.size()) {
            return false;
        }
        // 명령이 실제로 컨트롤러 밖으로 나가도록 flush.
        tcdrain(serial_fd_);
        return true;
    }

    // [2026-08-23 수정] 예전엔 이 함수가 한 사이클(10ms)당 딱 한 번 read()한
    // 청크를 그 자체로(줄 단위 분리 없이) parseAndPublish에 통째로 넘겼음 --
    // parseAndPublish는 문자열이 정확히 "$RPY,"로 시작해야만 파싱하므로,
    // 프레임이 두 번의 read() 사이에 걸쳐 끊기거나(잘려서 시작 부분을 못
    // 맞춤) 한 청크에 프레임이 여러 개 들어오면(뒤쪽 프레임들이 앞머리가
    // "$RPY,"가 아니게 됨) 조용히 유실됐다 -- EKF에 들어가는 orientation
    // 실효 주기가 하드웨어 자체 10Hz보다 더 들쭉날쭉해지는 원인 중 하나로
    // 추정(EKF 발산 진단 세션 참고). 이제 들어온 바이트를 rx_buffer_에
    // 누적하고, 개행 문자 기준으로 완전한 한 줄이 만들어질 때만 그 줄을
    // 꺼내 파싱한다 -- 프레임이 청크 경계에 걸려도 다음 read()에서 이어붙여
    // 완성되면 그때 파싱되므로 유실되지 않는다.
    void readSerialData()
    {
        char buf[256];
        int n = read(serial_fd_, buf, sizeof(buf) - 1);
        if (n <= 0) return;

        rx_buffer_.append(buf, static_cast<size_t>(n));

        // 파싱 안 되는 쓰레기(잘못된 보드레이트, 노이즈 등)가 개행 없이
        // 계속 들어오는 경우를 대비한 상한 -- 무한정 쌓이는 것 방지.
        constexpr size_t kMaxBufferBytes = 4096;
        if (rx_buffer_.size() > kMaxBufferBytes) {
            rx_buffer_.erase(0, rx_buffer_.size() - kMaxBufferBytes);
        }

        size_t newline_pos;
        while ((newline_pos = rx_buffer_.find('\n')) != std::string::npos) {
            std::string line = rx_buffer_.substr(0, newline_pos);
            rx_buffer_.erase(0, newline_pos + 1);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if (line.rfind("$RPY,", 0) == 0) {
                parseAndPublish(line);
            }
        }
    }

    void parseAndPublish(const std::string& raw_str)
    {
        // 1. 프로토콜 접두사 확인: "$RPY,<seq>,<roll_deg>,<pitch_deg>,<yaw_deg>*<checksum_hex>"
        if (raw_str.rfind("$RPY,", 0) != 0) {
            return;
        }

        // 2. '*' 기준으로 데이터부/체크섬부 분리 후 체크섬 검증
        size_t star_pos = raw_str.find('*');
        if (star_pos == std::string::npos || star_pos < 1 || star_pos + 2 >= raw_str.size()) {
            RCLCPP_WARN(this->get_logger(), "Malformed RPY frame (no checksum): %s", raw_str.c_str());
            return;
        }

        // 체크섬은 '$' 문자를 포함해 '*' 직전까지의 바이트를 XOR한 값
        // (실측 샘플 검증 결과, '$' 제외 시 체크섬이 일치하지 않음)
        std::string payload = raw_str.substr(1, star_pos - 1);
        uint8_t computed_checksum = 0;
        for (unsigned char c : raw_str.substr(0, star_pos)) {
            computed_checksum ^= c;
        }

        uint8_t received_checksum = 0;
        try {
            received_checksum = static_cast<uint8_t>(std::stoul(raw_str.substr(star_pos + 1, 2), nullptr, 16));
        }
        catch (...) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum field: %s", raw_str.c_str());
            return;
        }

        if (computed_checksum != received_checksum) {
            RCLCPP_WARN(this->get_logger(), "Checksum mismatch on IMU data: %s", raw_str.c_str());
            return;
        }

        // 3. ',' 기준 필드 파싱: [0]=RPY, [1]=seq, [2]=roll, [3]=pitch, [4]=yaw
        try {
            std::vector<std::string> fields;
            std::stringstream ss(payload);
            std::string field;
            while (std::getline(ss, field, ',')) {
                fields.push_back(field);
            }

            if (fields.size() < 5) {
                RCLCPP_WARN(this->get_logger(), "Incomplete RPY fields: %s", raw_str.c_str());
                return;
            }

            double roll_deg = std::stod(fields[2]);
            double pitch_deg = std::stod(fields[3]);
            double yaw_deg = std::stod(fields[4]);

            double roll_rad = roll_deg * M_PI / 180.0;
            double pitch_rad = pitch_deg * M_PI / 180.0;
            // [2026-08-25] yaw만 부호 반전. 실측 확인(로봇을 반시계로 90도
            // 돌렸는데 yaw가 +17.9deg -> -87.1deg로 계속 감소, RViz 회전
            // 방향도 실제와 반대로 보임): myAHRS+가 $RPY로 내는 yaw는
            // 시계=+ 관례(나침반/헤딩 스타일)인데, ROS(REP-103, 위에서 봤을 때
            // 반시계=+)/tf2::Quaternion::setRPY는 반시계=+를 가정하므로 그대로
            // 넣으면 부호가 반대로 들어감. roll/pitch는 이 테스트로 검증 안
            // 됐으므로(이미 실측 마운트 오프셋 보정이 들어가 있음,
            // ekf.launch.py의 base_to_imu_tf 참고) 건드리지 않음 -- 여기서
            // yaw만 반전.
            double yaw_rad = -yaw_deg * M_PI / 180.0;

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = this->now();
            imu_msg.header.frame_id = frame_id_;

            // 4. roll/pitch/yaw(라디안) -> quaternion 변환
            tf2::Quaternion q;
            q.setRPY(roll_rad, pitch_rad, yaw_rad);
            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            // orientation covariance 대각 성분 (정지 상태 실측 기반, 2000 샘플, imu_covariance_calibrator.py 사용)
            imu_msg.orientation_covariance[0] = orientation_covariance_roll_;
            imu_msg.orientation_covariance[4] = orientation_covariance_pitch_;
            imu_msg.orientation_covariance[8] = orientation_covariance_yaw_;

            // 5. $RPY 모드는 각속도/가속도 raw 값을 제공하지 않으므로 REP-145 관례에 따라 "데이터 없음" 명시
            imu_msg.angular_velocity_covariance[0] = -1.0;
            imu_msg.linear_acceleration_covariance[0] = -1.0;

            // 7. 정상 파싱 성공 시에만 publish
            imu_pub_->publish(imu_msg);
        }
        catch (...) {
            // 6. 숫자 변환 실패(std::stod 예외 등) 시 원본 로그 남기고 publish 생략
            RCLCPP_WARN(this->get_logger(), "Parsing error on IMU data stream: %s", raw_str.c_str());
        }
    }

    int serial_fd_;
    std::string rx_buffer_;  // readSerialData() 프레임 재조립용 누적 버퍼
    std::string frame_id_;
    double orientation_covariance_roll_;
    double orientation_covariance_pitch_;
    double orientation_covariance_yaw_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyAhrsDriverNode>());
    rclcpp::shutdown();
    return 0;
}