#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

static std::string format_signed3(const std::string& prefix, int value)
{
  std::ostringstream ss;
  ss << prefix
     << (value < 0 ? '-' : '+')
     << std::setw(3) << std::setfill('0') << std::abs(value);
  return ss.str();
}

static double apply_deadzone(double x, double dz)
{
  if (std::fabs(x) < dz) return 0.0;
  double s = (x > 0.0) ? 1.0 : -1.0;
  x = s * ((std::fabs(x) - dz) / (1.0 - dz));
  return x;
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
  , max_speed_deg_s_(60.0)       // D-pad=一定速度、アナログ=比例速度の上限 [deg/s]
  , min_deg_(-180.0)
  , max_deg_(180.0)
  , last_ud_i_(std::numeric_limits<int>::min())
  , last_rl_i_(std::numeric_limits<int>::min())
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("angle_cmd", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&AngleSendNode::joy_callback, this, std::placeholders::_1));

    last_tick_ = this->now();
    // 10ms周期で角度へ積分して送信（100Hz）
    timer_ = this->create_wall_timer(10ms, std::bind(&AngleSendNode::on_timer, this));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // アナログ軸（一般的に左スティック）
    double ax_h = (msg->axes.size() > 0) ? msg->axes[0] : 0.0; // 左(-) 〜 右(+)
    double ax_v = (msg->axes.size() > 1) ? msg->axes[1] : 0.0; // 下(-) 〜 上(+)

    // D-pad（ハットスイッチ）
    double hat_h = (msg->axes.size() > 6) ? msg->axes[6] : 0.0;
    double hat_v = (msg->axes.size() > 7) ? msg->axes[7] : 0.0;

    // デッドゾーン適用（アナログの微小ガタつき対策）
    ax_h = apply_deadzone(ax_h, 0.10);
    ax_v = apply_deadzone(ax_v, 0.10);

    // アナログ入力がほぼ0のときはD-padで置き換え（保持=一定速度）
    double cmd_h = (std::fabs(ax_h) > 1e-3) ? ax_h
                   : (hat_h > 0.5 ? 1.0 : (hat_h < -0.5 ? -1.0 : 0.0));
    double cmd_v = (std::fabs(ax_v) > 1e-3) ? ax_v
                   : (hat_v > 0.5 ? 1.0 : (hat_v < -0.5 ? -1.0 : 0.0));

    // 角速度コマンド [deg/s]（比例制御）
    vel_rl_deg_s_ = std::clamp(cmd_h, -1.0, 1.0) * max_speed_deg_s_;
    vel_ud_deg_s_ = std::clamp(cmd_v, -1.0, 1.0) * max_speed_deg_s_;
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

    // 送信用に整数化（必要なら小数対応に変更可）
    int rl_i = static_cast<int>(std::round(right_left_deg_));
    int ud_i = static_cast<int>(std::round(up_down_deg_));

    // 変化があったときのみ送信して帯域と負荷を抑制
    if (ud_i != last_ud_i_) {
      std_msgs::msg::String msg;
      msg.data = format_signed3("A0p", ud_i);
      pub_->publish(msg);
      last_ud_i_ = ud_i;
    }
    if (rl_i != last_rl_i_) {
      std_msgs::msg::String msg;
      msg.data = format_signed3("A0r", rl_i);
      pub_->publish(msg);
      last_rl_i_ = rl_i;
    }
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
  int last_ud_i_;
  int last_rl_i_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleSendNode>());
  rclcpp::shutdown();
  return 0;
}