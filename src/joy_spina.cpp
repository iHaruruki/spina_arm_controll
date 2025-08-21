#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <array>
#include <vector>
#include <string>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <cstdlib> // abs用

#define period 1

using namespace std::chrono_literals;

class AngleSendNode : public rclcpp::Node
{
public:
    AngleSendNode()
    : Node("angle_send_node"),
      id_(1), up_down_(0), right_left_(0)
    {
        pub_ = this->create_publisher<std_msgs::msg::String>("angle_cmd", 1);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&AngleSendNode::joy_callback, this, std::placeholders::_1));
    }

private:
    // ゼロ埋め3桁符号付き出力のユーティリティ関数
    std::string format_signed3(const std::string& prefix, int value)
    {
        std::ostringstream ss;
        if (value < 0) {
            ss << prefix << "-" << std::setw(3) << std::setfill('0') << std::abs(value);
        } else {
            ss << prefix << std::setw(3) << std::setfill('0') << value;
        }
        return ss.str();
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // axes[7]の配列外参照を防ぐ
        if (msg->axes.size() > 7 && msg->axes[7] == 1) {
            up_down_++;
            std::string str = format_signed3("A0p", up_down_);
            //RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
            std_msgs::msg::String out_msg;
            out_msg.data = str;
            pub_->publish(out_msg);
        }
        else if(msg->axes.size() > 7 && msg->axes[7] == -1) {
            up_down_--;
            std::string str = format_signed3("A0p", up_down_);
            //RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
            std_msgs::msg::String out_msg;
            out_msg.data = str;
            pub_->publish(out_msg);
        }

        if (msg->axes.size() > 7 && msg->axes[6] == 1) {
            right_left_++;
            std::string str = format_signed3("A0r", right_left_);
            //RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
            std_msgs::msg::String out_msg;
            out_msg.data = str;
            pub_->publish(out_msg);
        }
        else if (msg->axes.size() > 7 && msg->axes[6] == -1) {
            right_left_--;
            std::string str = format_signed3("A0r", right_left_);
            //RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
            std_msgs::msg::String out_msg;
            out_msg.data = str;
            pub_->publish(out_msg);
        }
        RCLCPP_INFO(this->get_logger(), "A0p: %s", format_signed3("A0p", up_down_).c_str());
        RCLCPP_INFO(this->get_logger(), "A0r: %s", format_signed3("A0r", right_left_).c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    int id_;
    int up_down_;
    int right_left_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleSendNode>());
    rclcpp::shutdown();
    return 0;
}