#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
#include <cmath>
#include <string>

// Soft current limiter -- shared by manual and autonomous drive.
//
// [하림 수정] 원래 "manual-drive-only"였는데, 이 노드 로직 자체는 명령 소스가
// 조이스틱이든 MPPI든 무관해서(그냥 /cmd_vel 스케일 다운) 자율주행에도 그대로
// 재사용한다. 입력 토픽만 파라미터로 뺐다 (수동: /cmd_vel_manual_raw 기본
// 유지, 자율: /cmd_vel_auto_raw로 launch에서 오버라이드).
//
// This is a SEPARATE layer from stability_monitor_node's 45A/3s hard
// cutoff (see control guide 3.3). That cutoff is the last line of
// defense against sustained overcurrent. This node instead reacts to
// brief current spikes -- expected during pivot turns on rubber tracks,
// where ground friction can spike torque current well before any
// sustained-overload condition -- by continuously and proportionally
// scaling down the commanded /cmd_vel, angular.z (rotation) first, so a
// spike self-corrects without ever commanding an abrupt stop. If this
// works as intended, the 45A/3s cutoff should rarely if ever trigger
// during manual driving.
class CurrentRampNode : public rclcpp::Node
{
public:
    CurrentRampNode() : Node("current_ramp_node"),
        left_current_(0.0), right_current_(0.0),
        vx_scale_(1.0), wz_scale_(1.0)
    {
        this->declare_parameter<double>("soft_current_limit_a", 22.0);
        this->declare_parameter<double>("ramp_down_gain", 0.05);   // scale/s per Amp over limit
        this->declare_parameter<double>("ramp_up_rate", 0.2);      // scale/s recovery
        // [2026-08-27] 50.0 -> 20.0 -- MPPI/reduced_odom 입력 체인(rmd_x8_driver,
        // myAHRS+)과 동일하게 20Hz로 통일하려는 사용자 요청.
        this->declare_parameter<double>("control_rate_hz", 20.0);
        // [하림 수정] 수동 기본값 유지, 자율주행 launch에서 /cmd_vel_auto_raw로 오버라이드
        this->declare_parameter<std::string>("cmd_vel_input_topic", "/cmd_vel_manual_raw");

        soft_current_limit_ = this->get_parameter("soft_current_limit_a").as_double();
        ramp_down_gain_ = this->get_parameter("ramp_down_gain").as_double();
        ramp_up_rate_ = this->get_parameter("ramp_up_rate").as_double();
        double rate_hz = this->get_parameter("control_rate_hz").as_double();
        period_s_ = 1.0 / rate_hz;
        // [2026-08-27] 로그 스로틀(아래 CURRENT_RAMP_SCALE)이 "1초마다"를
        // 유지하도록 rate_hz에서 직접 계산 -- 예전엔 50(=당시 고정 50Hz)을
        // 하드코딩해서 rate가 바뀌면 로그 주기도 같이 바뀌는 버그가 있었음.
        log_every_n_ticks_ = std::max(1L, std::lround(rate_hz));
        std::string cmd_vel_input_topic = this->get_parameter("cmd_vel_input_topic").as_string();

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_input_topic, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { last_cmd_ = *msg; });

        // rmd_x8_driver_node publishes best-effort; must match to receive it.
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/wheel/joint_states", rclcpp::QoS(10).best_effort(),
            std::bind(&CurrentRampNode::jointStateCallback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(period_s_),
            std::bind(&CurrentRampNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "current_ramp_node started: soft_limit=%.1fA, ramp_down_gain=%.3f, ramp_up_rate=%.3f",
            soft_current_limit_, ramp_down_gain_, ramp_up_rate_);
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size() && i < msg->effort.size(); ++i) {
            if (msg->name[i] == "left_wheel_joint") {
                left_current_ = msg->effort[i];
            } else if (msg->name[i] == "right_wheel_joint") {
                right_current_ = msg->effort[i];
            }
        }
    }

    void controlLoop()
    {
        double max_current = std::max(std::abs(left_current_), std::abs(right_current_));
        double excess = std::max(0.0, max_current - soft_current_limit_);

        if (excess > 0.0) {
            // Rotation is the dominant contributor to track-friction current
            // spikes, so drain it first; only touch forward/back speed if
            // draining rotation alone isn't enough.
            if (wz_scale_ > 0.0) {
                wz_scale_ -= period_s_ * ramp_down_gain_ * excess;
                wz_scale_ = std::max(0.0, wz_scale_);
            } else {
                vx_scale_ -= period_s_ * ramp_down_gain_ * excess;
                vx_scale_ = std::max(0.0, vx_scale_);
            }
        } else {
            wz_scale_ = std::min(1.0, wz_scale_ + period_s_ * ramp_up_rate_);
            vx_scale_ = std::min(1.0, vx_scale_ + period_s_ * ramp_up_rate_);
        }

        geometry_msgs::msg::Twist out;
        out.linear.x = last_cmd_.linear.x * vx_scale_;
        out.angular.z = last_cmd_.angular.z * wz_scale_;
        cmd_pub_->publish(out);

        if (excess > 0.0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "current_ramp: max_current=%.1fA (limit %.1fA) -> vx_scale=%.2f wz_scale=%.2f",
                max_current, soft_current_limit_, vx_scale_, wz_scale_);
        }

        // [계측, 2026-08-25] 순수 read-only 로그 -- excess>0(깎이는 중)일
        // 때만 찍히는 위 WARN과 달리, 회복 구간(scale<1.0이지만 지금 당장은
        // excess=0)도 놓치지 않고 보려는 용도. ramp_up_rate=0.2면 완전히
        // 깎인 뒤 회복에 최대 5초 걸리는데, 그 구간엔 지금까지 로그가 전혀
        // 없어서 "MPPI가 낸 값 그대로 안 나가고 있다"는 걸 알 방법이
        // 없었음. 매 tick 로직/발행에는 관여 안 함, 약 1초(log_every_n_ticks_회)마다만
        // stderr에 찍음.
        if (++log_tick_ % log_every_n_ticks_ == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[CURRENT_RAMP_SCALE] max_current=%.2fA vx_scale=%.3f wz_scale=%.3f "
                "cmd_in(vx=%.3f wz=%.3f) cmd_out(vx=%.3f wz=%.3f)",
                max_current, vx_scale_, wz_scale_,
                last_cmd_.linear.x, last_cmd_.angular.z, out.linear.x, out.angular.z);
        }
    }

    double soft_current_limit_;
    double ramp_down_gain_;
    double ramp_up_rate_;
    double period_s_;

    double left_current_;
    double right_current_;
    double vx_scale_;
    double wz_scale_;
    long log_tick_ = 0;  // [계측] CURRENT_RAMP_SCALE 로그 스로틀용
    long log_every_n_ticks_ = 20;  // rate_hz로부터 계산됨(생성자 참고), 기본값은 fallback일 뿐
    geometry_msgs::msg::Twist last_cmd_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CurrentRampNode>());
    rclcpp::shutdown();
    return 0;
}
