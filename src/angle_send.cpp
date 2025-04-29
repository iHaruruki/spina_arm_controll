#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <iomanip>
#include <sstream>
#include <chrono>

// 各モジュールの角度を設定(-30~30)
#define angle_m1 -10
#define angle_m2 -15
#define angle_m3 -25
#define angle_m4 -25
#define angle_m5 -25
#define angle_m6 -25

using namespace std::chrono_literals;

class AngleSendNode : public rclcpp::Node
{
public:
  AngleSendNode()
  : Node("angle_send_node"),
    id_(1),
    angles_{ angle_m1, angle_m2, angle_m3, angle_m4, angle_m5, angle_m6 }  // m1…m6 の角度
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("angle_cmd", 1);

    // 1秒ごとに publish_angle() を呼ぶタイマー
    timer_ = this->create_wall_timer(
      2s, std::bind(&AngleSendNode::publish_angle, this));
  }

private:
  void publish_angle()
  {
    if (id_ > 6) {
      RCLCPP_INFO(get_logger(), "All messages sent, shutting down.");
      rclcpp::shutdown();
      return;
    }

    // 例: "C1p010" / "C2p-010" のように 3 桁ゼロ埋めにしたい場合
    std::ostringstream ss;
    ss << 'C' << id_
       << 'p'
       << (angles_[id_-1] < 0 ? '-' : '+')
       << std::setw(3) << std::setfill('0')
       << std::abs(angles_[id_-1]);

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
  std::array<int,6> angles_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleSendNode>());
  return 0;
}
