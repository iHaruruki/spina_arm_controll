// src/serial_controller_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <array>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B2000000
#define BUFSIZE 11  // 送信バッファサイズ

class SerialController : public rclcpp::Node
{
public:
  SerialController()
  : Node("serial_controller")
  {
    // --- シリアルポートをオープン ---
    fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open %s", SERIAL_PORT);
      throw std::runtime_error("serial open failed");
    }

    // --- 既存のターミナル設定を保持 ---
    tcgetattr(fd_, &oldtio_);

    // --- シリアルポート設定 ---
    struct termios tio{};
    cfsetispeed(&tio, BAUDRATE);
    cfsetospeed(&tio, BAUDRATE);
    tio.c_cflag = CS8 | CREAD | CLOCAL;
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tcflush(fd_, TCIFLUSH);
    tcsetattr(fd_, TCSANOW, &tio);

    // --- トピック購読者を作成 ---
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "angle_commands", 10,
      std::bind(&SerialController::on_command, this, std::placeholders::_1));

    // 購読成功をログに出力
    RCLCPP_INFO(this->get_logger(), "Subscribed to 'angle_commands' successfully.");
    RCLCPP_INFO(this->get_logger(), "SerialController node started");
  }

  ~SerialController()
  {
    // ノード終了時に元のターミナル設定に戻す
    tcsetattr(fd_, TCSANOW, &oldtio_);
    close(fd_);
    RCLCPP_INFO(this->get_logger(), "SerialController node shut down, serial port closed.");
  }

private:
  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto &s = msg->data;
    // 受信データをログに表示
    RCLCPP_INFO(this->get_logger(), "Received topic data: '%s'", s.c_str());

    if (s.size() < 7) {
      RCLCPP_WARN(this->get_logger(), "Invalid command length: '%s'", s.c_str());
      return;
    }

    char buf0 = s[0];
    int sign = (s[3] == '-') ? -1 : +1;
    int angle = sign * (((s[4]-'0')*100) + ((s[5]-'0')*10) + (s[6]-'0'));

    std::array<uint8_t, BUFSIZE> tx{};
    tx[0] = 0xAA;
    tx[1] = 0xC6;
    tx[2] = 0x00;
    tx[3] = 0x00;

    if (buf0 == 'C' && -30 <= angle && angle <= 30) {
      // モジュール単体制御コマンドを構築
      tx[4] = 'C';        // コマンド識別子
      tx[5] = s[1];       // モジュール番号
      tx[6] = s[2];       // 'p'
      tx[7] = s[3];       // 符号
      int a = abs(angle);
      tx[8] = char((a/10) + '0');
      tx[9] = char((a%10) + '0');
      tx[10] = 0x55;      // 終端

      // シリアルに送信
      write(fd_, tx.data(), tx.size());
      RCLCPP_INFO(this->get_logger(), "Sent module command: '%c%c%c%02d'",
                  tx[5], tx[6], tx[7], a);
    }
    else if (buf0 == 'A' && -180 <= angle && angle <= 180) {
      // 全体制御：6つのモジュールに順次送信
      for (int i = 0; i < 6; ++i) {
        tx[4] = 'C';
        tx[5] = char('1' + i);
        tx[6] = s[2];
        tx[7] = s[3];
        int a = abs(angle);
        tx[8] = char((a/60) + '0');
        tx[9] = char(((a/6) - ((a/60)*10)) + '0');
        tx[10] = 0x55;

        write(fd_, tx.data(), tx.size());
        RCLCPP_INFO(this->get_logger(), "Sent all-mod #%d: '%c%c%02d'", i+1, s[2], s[3], a);
        usleep(50000);
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Angle out of range or bad command: %d", angle);
    }
  }

  int fd_;
  struct termios oldtio_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
