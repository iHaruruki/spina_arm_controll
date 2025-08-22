#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

static std::string format_signed3(const std::string& prefix, int value)
{
  std::ostringstream ss;
  ss << prefix
     << (value < 0 ? '-' : '+')
     << std::setw(3) << std::setfill('0') << std::abs(value);
  return ss.str();
}

class AngleSendNode : public rclcpp::Node
{
public:
  AngleSendNode()
  : Node("angle_send_node")
  , up_down_deg_(0.0)
  , right_left_deg_(0.0)
  , vel_ud_deg_s_(0.0)
  , vel_rl_deg_s_(0.0)
  , max_speed_deg_s_(60.0)   // D-pad保持中の角速度[deg/s]（調整可）
  , min_deg_(-180.0)
  , max_deg_(180.0)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("angle_cmd", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&AngleSendNode::joy_callback, this, std::placeholders::_1));

    last_tick_ = this->now();
    // 10msごと（100Hz）に必ず送信
    timer_ = this->create_wall_timer(10ms, std::bind(&AngleSendNode::on_timer, this));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // D-pad（ハット）だけを使用
    double hat_h = (msg->axes.size() > 6) ? msg->axes[6] : 0.0; // 左(-1) / 右(+1)
    double hat_v = (msg->axes.size() > 7) ? msg->axes[7] : 0.0; // 下(-1) / 上(+1)

    auto hat_to_cmd = [](double v) {
      if (v > 0.5)  return 1.0;
      if (v < -0.5) return -1.0;
      return 0.0;
    };

    double cmd_h = hat_to_cmd(hat_h);
    double cmd_v = hat_to_cmd(hat_v);

    // 角速度コマンド [deg/s]（D-pad保持 = 一定速度）
    vel_rl_deg_s_ = cmd_h * max_speed_deg_s_;
    vel_ud_deg_s_ = cmd_v * max_speed_deg_s_;
  }

  void on_timer()
  {
    // 経過時間[s]
    rclcpp::Time now = this->now();
    double dt = (now - last_tick_).seconds();
    last_tick_ = now;
    if (dt <= 0.0 || dt > 0.2) {
      // スリープ復帰などで異常に大きいdtはスキップ
      return;
    }

    // 角度へ積分
    right_left_deg_ += vel_rl_deg_s_ * dt;
    up_down_deg_    += vel_ud_deg_s_ * dt;

    // 範囲クリップ
    right_left_deg_ = std::clamp(right_left_deg_, min_deg_, max_deg_);
    up_down_deg_    = std::clamp(up_down_deg_,    min_deg_, max_deg_);

    // 送信（A0p/A0rを毎周期送る）
    std_msgs::msg::String msg1;
    msg1.data = format_signed3("A0p", static_cast<int>(std::round(up_down_deg_)));
    pub_->publish(msg1);

    std_msgs::msg::String msg2;
    msg2.data = format_signed3("A0r", static_cast<int>(std::round(right_left_deg_)));
    pub_->publish(msg2);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_tick_;

  // 内部状態
  double up_down_deg_;
  double right_left_deg_;
  double vel_ud_deg_s_;
  double vel_rl_deg_s_;
  const double max_speed_deg_s_;
  const double min_deg_, max_deg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleSendNode>());
  rclcpp::shutdown();
  return 0;
}