#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <chrono>

// 各モジュールの姿勢の軸設定('p' or 'r')
#define posture_m1 'r'
#define posture_m2 'r'
#define posture_m3 'r'
#define posture_m4 'r'
#define posture_m5 'r'
#define posture_m6 'r'

// 各モジュールの角度を設定(-30~30)
#define angle_m1 -10
#define angle_m2 -10
#define angle_m3 -10
#define angle_m4 -20
#define angle_m5 -20
#define angle_m6 -20
// publish_angle() を呼ぶタイマーの周期(s)
#define period 1

using namespace std::chrono_literals;

class AngleSendNode : public rclcpp::Node
{
public:
    AngleSendNode()
    : Node("angle_send_node"),
      id_(1),
      angles_{ angle_m1, angle_m2, angle_m3, angle_m4, angle_m5, angle_m6 },
      axes_{ posture_m1, posture_m2, posture_m3, posture_m4, posture_m5, posture_m6 }
    {
        pub_ = this->create_publisher<std_msgs::msg::String>("angle_cmd", 1);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(period),
            std::bind(&AngleSendNode::publish_angle, this));
    }

private:
    void publish_angle()
    {
        if (id_ > 6) {
            RCLCPP_INFO(get_logger(), "All messages sent, shutting down.");
            rclcpp::shutdown();
            return;
        }

        std::ostringstream ss;
        ss << 'C' << id_
           << axes_[id_ - 1]
           << (angles_[id_ - 1] < 0 ? '-' : '+')
           << std::setw(3) << std::setfill('0')
           << std::abs(angles_[id_ - 1]);

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();

        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Publish: %s (module:%d)",
                     msg.data.c_str(), id_);

        ++id_;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int id_;
    std::array<int, 6> angles_;
    std::array<char, 6> axes_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleSendNode>());
    rclcpp::shutdown();
    return 0;
}
