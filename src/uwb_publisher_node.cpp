#include <libserial/SerialPort.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <algorithm>

class UWBPublisher : public rclcpp::Node {
public:
    UWBPublisher() : Node("uwb_publisher"), serial_port("/dev/ttyUSB0"), publisher_(nullptr) {
        this->declare_parameter<int>("baud_rate", 115200);
        int baud_rate = this->get_parameter("baud_rate").as_int();

        serial_port.Open();
        serial_port.SetBaudRate(baud_rate);
        serial_port.SetCharacterSize(LibSerial::SerialPort::CharacterSize::CHAR_SIZE_8);
        serial_port.SetParity(LibSerial::SerialPort::Parity::PARITY_NONE);
        serial_port.SetStopBits(LibSerial::SerialPort::StopBits::STOP_BITS_1);

        publisher_ = this->create_publisher<std_msgs::msg::String>("uwb_data", 10);

        // Start asynchronous read
        startAsyncRead();
    }

private:
    void startAsyncRead() {
        serial_port.SetCallback(readCallback, this);
        serial_port.StartReading();
    }

    static void readCallback(LibSerial::SerialPort& serial_port, char* buffer, size_t size, void* user_data) {
        UWBPublisher* publisher = static_cast<UWBPublisher*>(user_data);
        std_msgs::msg::String msg;
        msg.data = std::string(buffer, size);

        // Remove spaces, carriage returns, and newlines from msg.data
        msg.data.erase(std::remove_if(msg.data.begin(), msg.data.end(), [](char c) {
            return c == ' ' || c == '\r' || c == '\n';
        }), msg.data.end());
        if (msg.data != "" && msg.data.size() <= 20)
            publisher->publisher_->publish(msg);
    }

    LibSerial::SerialPort serial_port;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBPublisher>());
    rclcpp::shutdown();
    return 0;
}
