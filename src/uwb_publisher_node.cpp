#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <numeric> // for std::accumulate
#include <std_msgs/msg/string.hpp>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

class UWBPublisher : public rclcpp::Node {
public:
  UWBPublisher()
      : Node("uwb_publisher"), publisher_(nullptr), serial_fd_(-1),
        thread_stop_(false) {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);

    std::string serial_port = this->get_parameter("serial_port").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();

    serial_fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      rclcpp::shutdown();
      return;
    }
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error in tcgetattr");
      close(serial_fd_);
      rclcpp::shutdown();
      return;
    }
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error in tcsetattr");
      close(serial_fd_);
      rclcpp::shutdown();
      return;
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("uwb_data", 10);

    serial_thread_ = std::thread(&UWBPublisher::serialReadThread, this);
  }

  ~UWBPublisher() {
    thread_stop_ = true;
    if (serial_thread_.joinable())
      serial_thread_.join();

    if (serial_fd_ != -1)
      close(serial_fd_);
  }

private:
  void serialReadThread() {
    char buffer[1024];

    while (!thread_stop_) {
      int bytes_available;
      ioctl(serial_fd_, FIONREAD, &bytes_available);
      if (bytes_available > 0) {
        ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer));
        if (bytes_read > 0) {
          std::string data(buffer, bytes_read);
          std_msgs::msg::String msg;
          msg.data = data;
          publisher_->publish(msg);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int serial_fd_;
  std::thread serial_thread_;
  std::atomic<bool> thread_stop_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UWBPublisher>());
  rclcpp::shutdown();
  return 0;
}
