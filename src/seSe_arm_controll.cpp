#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:
  ArmController()
  : Node("arm_controller")
  {
    // シリアルポートをオープン
    fd_ = open(SERIAL_PORT, O_RDWR);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", SERIAL_PORT);
      rclcpp::shutdown();
      return;
    }
    // 通信設定
    struct termios newtio;
    ioctl(fd_, TCGETS, &oldtio_);
    newtio.c_cflag = BAUDRATE | CS8 | CREAD;
    tcsetattr(fd_, TCSANOW, &newtio);
    ioctl(fd_, TCSETS, &newtio);

    // トピック購読
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "angle_cmd", 10,
      std::bind(&ArmController::cmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArmController node started");
  }

  ~ArmController()
  {
    // シリアル設定を元に戻してクローズ
    ioctl(fd_, TCSETS, &oldtio_);
    close(fd_);
  }

private:
  void cmdCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string &buf = msg->data;
    if (buf.size() < 7) {
      RCLCPP_WARN(this->get_logger(), "Invalid command length");
      return;
    }

    // 受信コマンド例: "A0p-030" や "C1y015"
    int angle = ((buf[4]-'0')*100) + ((buf[5]-'0')*10) + (buf[6]-'0');
    unsigned char send[11] = {0xAA, 0xC6, 0x00, 0x00};

    if (buf[0]=='C' && -30 <= angle && angle <= 30) {
      // モジュール単体制御
      send[4] = 'C';
      send[5] = buf[1];
      send[6] = buf[2];
      send[7] = buf[3];
      send[8] = (angle/10) + '0';
      send[9] = (angle%10) + '0';
      send[10] = 0x55;
      write(fd_, send, sizeof(send));
      RCLCPP_INFO(this->get_logger(), "Sent C command: %s", buf.c_str());

    } else if (buf[0]=='A' && -180 <= angle && angle <= 180) {
      // 全体角度制御
      for (int i = 0; i < 6; i++) {
        send[4] = 'C';
        send[5] = '1' + i;       // モジュール番号
        send[6] = buf[2];
        send[7] = buf[3];
        send[8] = (angle/60) + '0';
        send[9] = ((angle/6)%10) + '0';
        send[10] = 0x55;
        write(fd_, send, sizeof(send));
        RCLCPP_INFO(this->get_logger(), "Sent A command to module %d: %s", i+1, buf.c_str());
        usleep(50000);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid value or out of range: %s", buf.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  int fd_;
  struct termios oldtio_;

  static constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";
  static constexpr int BAUDRATE = B2000000;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}
