#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>

// [하림 수정, 2026-08-23] 자세(roll/pitch) 긴급정지 + 과전류 보호 제거.
//
// 근거:
//   - 자세 긴급정지: manual 주행 실측으로 전복 가능성이 낮다고 판단, 그리고
//     이 임계값(critical_roll=20deg)이 (당시) imu_slope_mode_node의 정렬
//     진입 임계값(slope_enter_threshold_deg=22deg)보다 낮아서 -- 완만하게
//     대응해야 할 상황(ALIGNING)이 시작되기도 전에 이 노드가 먼저 긴급정지를
//     걸어버리는 순서 역전 문제도 있었음. 경사 대응 상태머신 쪽이 전담하는
//     것으로 정리(imu_slope_mode_node는 이후 slope_traverse_node로 대체됨,
//     이 정리 자체는 그대로 유효).
//   - 과전류 보호(45A/3초): manual 주행에서 하중이 눌리는 상황까지 이미
//     충분히 시험해서 위험 소지가 낮다고 판단.
//
// 남긴 것: 구동 모터 통신두절/하드웨어 에러(STALE/ERROR) 감지 -> 즉시 정지.
// 이건 rmd_x8_driver_node의 comm_timeout_protection_ms(드라이버->모터
// 방향 하드웨어 워치독)나 cmd_vel_timeout_s(ROS->드라이버 방향)와 겹치지
// 않는다 -- 그 둘은 "명령이 안 옴"을 잡는 것이고, 이건 반대로 "모터 피드백이
// 끊기거나 에러를 보고하는데 명령은 계속 들어오는" 상황을 잡는다.
// rmd_x8_driver_node 자신은 이 상황에서 진단 토픽에 STALE/ERROR를 보고만
// 하고 명령 실행 자체는 그대로 계속하므로(코드 확인됨), 이걸 잡아서 멈추는
// 안전장치가 이것 하나뿐이라 남겨둔다.
//
// 이 노드가 자세 판단을 안 하게 되면서, /cmd_vel_safety에 자세 기준으로
// 발행하는 노드가 (2026-08-26 기준) slope_traverse_node 하나만 남는다 --
// 두 노드가 같은 토픽에 동시에 발행하며 우선순위 없이 경합하던 문제가
// 이걸로 해소됨. 모터 폴트 발행은 orthogonal한 하드웨어 안전 케이스라
// (발동하면 그 자체로 최우선이어야 하는 게 맞음) slope_traverse_node와
// 경합해도 문제 없음.
//
// [2026-08-23 추가] fault-latch를 "영구(재시작해야만 해제)"에서 "N초 연속
// 정상이면 자동 해제"로 변경. 실기에서 CAN이 잠깐 끊겼다 스스로 복구된
// 상황을 겪었는데, 영구 래치라 CAN은 이미 정상인데도 autonomous.launch.py
// 전체(=EKF 포함)를 재시작해야만 다시 움직일 수 있었음 -- 불필요하게 큰
// 마찰. 그렇다고 "한 번이라도 정상으로 보이면 즉시 해제"로 가면 원래
// 영구 래치를 도입했던 이유(연결이 헐거워서 "떴다 안 떴다" 하는 상황에서
// 순간적으로 좋아 보인 한 프레임만 보고 신뢰해버리는 것)가 재발한다. 그래서
// 그 중간 -- fault_clear_dwell_sec(기본 4초) 동안 끊김 없이 계속 정상이어야
// 해제. 짧은 플리커는 안 풀리고, 진짜 복구는 자동으로 풀림.
class StabilityMonitorNode : public rclcpp::Node
{
public:
    StabilityMonitorNode() : Node("stability_monitor_node"),
      drive_motor_fault_latched_(false)
    {
        fault_clear_dwell_sec_ = this->declare_parameter<double>("fault_clear_dwell_sec", 4.0);

        // RMD-X8 구동 모터 상태 피드백 구독: rmd_x8_driver_node가 실제로
        // 발행하는 /wheel/motor_status를 직접 구독.
        motor_status_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
            "/wheel/motor_status", 10,
            std::bind(&StabilityMonitorNode::motorStatusCallback, this, std::placeholders::_1));

        // 안전 비상 개입 전용 퍼블리셔
        safety_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safety", 10);

        // 100Hz 고속 제어 루프 (지연 시간 최소화)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&StabilityMonitorNode::controlLoop, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Stability monitor (drive motor fault-latch only, auto-clear after %.1fs clean) "
            "initialized -- attitude/overcurrent emergency-stop removed (2026-08-23), see class "
            "docstring.",
            fault_clear_dwell_sec_);
    }

private:
    void motorStatusCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
    {
        // [수정] 구동 모터 하드웨어 에러/피드백 두절(STALE) 시 즉시 위험 상태로
        // 취급 + 래치(latch, 클래스 상단 docstring 참고 -- 2026-08-23부터
        // "N초 연속 정상"이어야 자동 해제되는 방식).
        bool has_fault_this_msg = false;
        for (const auto & status : msg->status) {
            if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR ||
                status.level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
                has_fault_this_msg = true;
                // [2026-08-19] 스로틀 추가. 이 콜백이 /wheel/motor_status 발행
                // 주기(rmd_x8_driver_node 기준 최대 50Hz)마다 불려서, 모터
                // 전원이 꺼진 채로 방치되면(배터리 OFF 등) 매 사이클 ERROR
                // 로그가 쏟아져 터미널이 스팸으로 뒤덮이는 문제 발견-- 실제
                // 상태 판단(drive_motor_fault_latched_)은 걸린 동안 그대로
                // 유지하고, 로그만 2초에 한 번으로 줄임.
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "[%s] fault: %s", status.name.c_str(), status.message.c_str());
            }
        }

        if (has_fault_this_msg) {
            drive_motor_fault_latched_ = true;
            fault_clear_ok_since_.reset();  // 정상 연속 기록 리셋
            return;
        }

        if (!drive_motor_fault_latched_) {
            return;  // 원래도 정상 -- 할 일 없음
        }

        // 이번 메시지엔 fault가 없음 -- "계속 정상"인 시간을 재서
        // fault_clear_dwell_sec_ 이상 끊김 없이 유지되면 자동 해제.
        const rclcpp::Time now = this->get_clock()->now();
        if (!fault_clear_ok_since_) {
            fault_clear_ok_since_ = now;
        }
        if ((now - *fault_clear_ok_since_).seconds() >= fault_clear_dwell_sec_) {
            drive_motor_fault_latched_ = false;
            fault_clear_ok_since_.reset();
            RCLCPP_WARN(
                this->get_logger(),
                "Drive motor fault auto-cleared after %.1fs of clean feedback.",
                fault_clear_dwell_sec_);
        }
    }

    void controlLoop()
    {
        // 모터 통신두절/하드웨어 에러 외에는 이 노드가 개입할 이유가 없음
        // (자세/과전류 판단 제거, 클래스 상단 docstring 참고).
        if (!drive_motor_fault_latched_) {
            return;
        }

        // 모터 상태 자체를 못 믿는 상황(통신 두절/하드웨어 에러)에서는
        // "저속으로라도 탈출"이라는 논리가 성립하지 않는다 -- 명령이 실제로
        // 어떻게 실행될지 보장이 없으므로 무조건 0.
        geometry_msgs::msg::Twist safety_twist;
        safety_twist.linear.x = 0.0;
        safety_twist.angular.z = 0.0;
        safety_cmd_pub_->publish(safety_twist);
    }

    // [수정] 래치형이지만 영구는 아님 -- fault_clear_dwell_sec_ 동안 끊김
    // 없이 계속 정상이면 자동으로 false로 돌아간다 (motorStatusCallback 참고).
    bool drive_motor_fault_latched_;
    double fault_clear_dwell_sec_;
    std::optional<rclcpp::Time> fault_clear_ok_since_;

    // ROS 2 인터페이스
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr motor_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safety_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StabilityMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
