#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <chrono>

using namespace std::chrono_literals;

class AngleSendNode : public rclcpp::Node
{
public:
  AngleSendNode()
  : Node("angle_send_node"), id_(1)
  {
    // --- 1) modules パラメータ宣言 ---
    std::vector<std::string> default_modules = {
      "p-0", "p-0", "p-0",
      "p-0", "p-0", "p-0"
    };
    this->declare_parameter<std::vector<std::string>>("modules", default_modules);

    // --- 2) get_parameter で取得 ---
    std::vector<std::string> modules_param;
    if (!this->get_parameter("modules", modules_param)) {
      RCLCPP_ERROR(get_logger(), "Failed to get 'modules' parameter");
      rclcpp::shutdown();
      return;
    }

    if (modules_param.size() != 6) {
      RCLCPP_ERROR(get_logger(),
        "'modules' must be an array of 6 strings (e.g. ['r-10','p+20',...])");
      rclcpp::shutdown();
      return;
    }

    // --- 3) パースして内部配列に格納 ---
    for (size_t i = 0; i < 6; ++i) {
      const auto &s = modules_param[i];
      if (s.size() < 2) {
        RCLCPP_ERROR(get_logger(),
          "modules[%zu] invalid: '%s'", i, s.c_str());
        rclcpp::shutdown();
        return;
      }
      axes_[i] = s[0];
      try {
        angles_[i] = std::stoi(s.substr(1));
      } catch (...) {
        RCLCPP_ERROR(get_logger(),
          "modules[%zu] angle part not a number: '%s'", i, s.c_str());
        rclcpp::shutdown();
        return;
      }
    }

    // --- 4) publisher & timer setup ---
    pub_ = this->create_publisher<std_msgs::msg::String>("angle_cmd", 1);
    timer_ = this->create_wall_timer(
      1s, std::bind(&AngleSendNode::publish_angle, this));
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
       << axes_[id_-1]
       << (angles_[id_-1] < 0 ? '-' : '+')
       << std::setw(3) << std::setfill('0')
       << std::abs(angles_[id_-1]);

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    pub_->publish(msg);

    RCLCPP_INFO(get_logger(),
                "Publish: %s (module:%d)",
                msg.data.c_str(), id_);
    ++id_;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int id_;
  std::array<int,6>     angles_;
  std::array<char,6>    axes_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleSendNode>());
  rclcpp::shutdown();
  return 0;
}
