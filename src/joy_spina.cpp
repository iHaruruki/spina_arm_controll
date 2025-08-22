#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <array>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <cstdlib> // abs

using namespace std::chrono_literals;

class AngleSendNode : public rclcpp::Node
{
public:
    AngleSendNode()
    : Node("angle_send_node"),
      up_down_(0),
      right_left_(0)
    {
        // パラメータ宣言（起動時に --ros-args -p で上書き可能）
        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0); // 定期送信の周波数
        step_            = this->declare_parameter<int>("step", 2);                  // 十字キー1回の増分

        pub_ = this->create_publisher<std_msgs::msg::String>("angle_cmd", rclcpp::QoS(10).reliable());

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::SensorDataQoS(),
            std::bind(&AngleSendNode::joy_callback, this, std::placeholders::_1));

        // タイマー（publish_rate_hz_ に基づく周期）
        auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / std::max(1e-6, publish_rate_hz_)));
        timer_ = this->create_wall_timer(period, std::bind(&AngleSendNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "angle_cmd will be published at %.2f Hz (step=%d)",
                    publish_rate_hz_, step_);
    }

private:
    // -999〜+999に丸めて "A0x+003" 形式にする
    static int clamp3(int v) {
        if (v >  180) return  180;
        if (v < -180) return -180;
        return v;
    }

    std::string format_signed3(const std::string& prefix, int value)
    {
        std::ostringstream ss;
        ss << prefix
           << (value < 0 ? '-' : '+')
           << std::setw(3) << std::setfill('0') << std::abs(value);
        return ss.str();
    }

    void publish_value(const char* prefix, int value)
    {
        std_msgs::msg::String out;
        out.data = format_signed3(prefix, clamp3(value));
        pub_->publish(out);
    }

    // 定期送信（毎周期、現在値を2本とも送る）
    void on_timer()
    {
        publish_value("A0p", up_down_);
        publish_value("A0r", right_left_);
        // ログはウルサくなりがちなのでスロットル
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "tick A0p=%s  A0r=%s",
                             format_signed3("A0p", up_down_).c_str(),
                             format_signed3("A0r", right_left_).c_str());
    }

    // Joy入力で状態だけ更新（送信はしない）
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // 配列長チェック（axes[6], axes[7] を使う）
        if (msg->axes.size() > 7) {
            // 上: axes[7] == 1, 下: == -1
            if (msg->axes[7] == 1)   up_down_   -= step_;
            else if (msg->axes[7] == -1) up_down_   += step_;

            // 左: axes[6] == 1, 右: == -1
            if (msg->axes[6] == 1)   right_left_ -= step_;
            else if (msg->axes[6] == -1) right_left_ += step_;
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int up_down_;
    int right_left_;
    double publish_rate_hz_;
    int step_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleSendNode>());
    rclcpp::shutdown();
    return 0;
}
