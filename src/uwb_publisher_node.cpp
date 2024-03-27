#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <deque>
#include <numeric> // for std::accumulate
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class UWBPublisher : public rclcpp::Node {
public:
    UWBPublisher() : Node("uwb_publisher"), publisher_(nullptr), serial_fd_(-1), thread_stop_(false) {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        serial_fd_ = open(serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ == -1) {
            std::cerr << "Failed to open serial port" << std::endl;
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_fd_, &tty) != 0) {
            std::cerr << "Error in tcgetattr" << std::endl;
            close(serial_fd_);
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
            std::cerr << "Error in tcsetattr" << std::endl;
            close(serial_fd_);
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
        std::deque<int> range_values;
        const size_t filter_size = 15;

        while (!thread_stop_) {
            int bytes_available;
            ioctl(serial_fd_, FIONREAD, &bytes_available);
            if (bytes_available > 0) {
                ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer));
                if (bytes_read > 0) {
                    std::string data(buffer, bytes_read);
                    if (data.size() == 13) {
                        int id = std::stoi(data.substr(0, 4));
                        int range = std::stoi(data.substr(4, 5));
                        int rx_power = std::stoi(data.substr(9, 4));    

                        range_values.push_back(range);
                        if (range_values.size() > filter_size) {
                            range_values.pop_front();
                        }

                        int sum = std::accumulate(range_values.begin(), range_values.end(), 0);
                        int average_range = range_values.size() > 0 ? sum / range_values.size() : 0;
                        
                        char buffer[100];
                        snprintf(bufferi sizeof(buffer), "%04d%05d%04d", id, average_range, rx_power);

                        std_msgs::msg::String msg;
                        msg.data = average_range;        
                        publisher_->publish(msg);
                    }
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBPublisher>());
    rclcpp::shutdown();
    return 0;
}