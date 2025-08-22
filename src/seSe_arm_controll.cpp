#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

class ArmController : public rclcpp::Node
{
public:
  ArmController()
  : Node("arm_controller")
  {
    fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", SERIAL_PORT);
      rclcpp::shutdown();
      return;
    }

    // 現在設定の保存
    if (tcgetattr(fd_, &oldtio_) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcgetattr failed");
      rclcpp::shutdown();
      return;
    }

    // RAWモード設定
    struct termios newtio;
    std::memset(&newtio, 0, sizeof(newtio));
    cfmakeraw(&newtio);
    cfsetispeed(&newtio, BAUDRATE);
    cfsetospeed(&newtio, BAUDRATE);
    newtio.c_cflag |= (CLOCAL | CREAD | CS8);
    // ハード/ソフトフロー制御OFF
#ifdef CRTSCTS
    newtio.c_cflag &= ~CRTSCTS;
#endif
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

    newtio.c_cc[VMIN]  = 0;
    newtio.c_cc[VTIME] = 1; // 0.1s

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &newtio) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcsetattr failed");
      rclcpp::shutdown();
      return;
    }

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "angle_cmd", 50,
      std::bind(&ArmController::cmdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ArmController node started");
  }

  ~ArmController()
  {
    tcsetattr(fd_, TCSANOW, &oldtio_);
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

    // ±ddd 形式を想定（publisher側で常に符号付け）
    int sign = (buf[3] == '-') ? -1 : 1;
    int angle = ((buf[4]-'0')*100) + ((buf[5]-'0')*10) + (buf[6]-'0');
    angle *= sign;

    char send[11];

    if (buf[0]=='C' && -30 <= angle && angle <= 30) {
      send[0] = 0xAA;
      send[1] = 0xC6;
      send[2] = 0x00;
      send[3] = 0x00;
      send[4] = 'C';
      send[5] = buf[1];
      send[6] = buf[2];
      send[7] = buf[3]; // 符号
      send[8] = (std::abs(angle)/10) + '0';
      send[9] = (std::abs(angle) - ((std::abs(angle)/10) * 10)) + '0';
      send[10] = 0x55;
      write(fd_, send, sizeof(send));
      RCLCPP_INFO(this->get_logger(), "Sent command: %s", buf.c_str());
    }
    else if (buf[0]=='A' && -180 <= angle && angle <= 180) {
      // 全体角度制御：各モジュールへ短いインターバルで送信
      for (int i = 0; i < 6; i++) {
        send[0] = 0xAA;
        send[1] = 0xC6;
        send[2] = 0x00;
        send[3] = 0x00;
        send[4] = 'C';
        send[5] = i+1+'0';
        send[6] = buf[2];
        send[7] = buf[3]; // 符号
        // 角度→プロトコルの2桁表現（既存ロジックを踏襲）
        int a = std::abs(angle);
        send[8] = (a/60) + '0';
        send[9] = (a/6 - ((a/60) * 10)) + '0';
        send[10] = 0x55;
        write(fd_, send, sizeof(send));
        RCLCPP_INFO(this->get_logger(), "Sent command to module %d: %s", i+1, buf.c_str());

        // 可能なら0〜2msまで短縮（ハードが許す範囲で）
        usleep(2000);
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Invalid value or out of range: %s", buf.c_str());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  int fd_;
  struct termios oldtio_;

  static constexpr const char* SERIAL_PORT = "/dev/ttyUSB0";
  static constexpr speed_t BAUDRATE = B2000000;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmController>());
  rclcpp::shutdown();
  return 0;
}