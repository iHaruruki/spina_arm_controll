#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AngleCmdPublisher : public rclcpp::Node
{
public:
    AngleCmdPublisher()
        : Node("angle_cmd_publisher"), messages_(
            {   "C2p-020", "C3p-020", "C4p-020", "C5p-020", "A0p-000", "A0r-000"
                "C2r+020", "C3r+020", "C4r+020", "C5r+020", "A0p-000", "A0r-000"
                "C2p+020", "C3p+020", "C4p+020", "C5p+020", "A0p-000", "A0r-000"
                "C2r-020", "C3r-020", "C4r-020", "C5r-020", "A0p-000", "A0r-000"
            }), current_index_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/angle_cmd", 10);
        timer_ = this->create_wall_timer(2s, std::bind(&AngleCmdPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = messages_[current_index_];
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        // Move to the next message in the list
        current_index_ = (current_index_ + 1) % messages_.size();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::string> messages_;
    size_t current_index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleCmdPublisher>());
    rclcpp::shutdown();
    return 0;
}
