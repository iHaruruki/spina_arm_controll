#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

// 各モジュールの角度を設定(-30~30)
#define angle_m1 10
#define angle_m2 -10
#define angle_m3 10
#define angle_m4 -10
#define angle_m5 10
#define angle_m6 10

int main(int argc,char **argv)
{
    // Initialize Node
    rclcpp::init(argc,argv);
    // Create Node
    auto node =rclcpp::Node::make_shared("angle_send_node");
    // Create Topic
    auto pub_cmd = node->create_publisher<std_msgs::msg::String>("angle_cmd", 1);

    //ループ周期の設定
    rclcpp::WallRate loop(0.5); //0.5Hz

    int id = 0;
    int angle[6] = {angle_m1, angle_m2, angle_m3, angle_m4, angle_m5, angle_m6};

    //各モジュールに角度を送信
    while(rclcpp::ok())
    {
        id++;
        auto msg = std_msgs::msg::String();
        msg.data = std::string("C") + std::to_string(id) + "p" + std::to_string(angle[id-1]);
        pub_cmd->publish(msg);
        RCLCPP_INFO(node->get_logger(), "Publish: %s (module:%d)",msg.data.c_str(),id);
        if(id == 6) break;
        loop.sleep();
    }

    // Shutdown Connection
    rclcpp::shutdown();
    return 0;
}
