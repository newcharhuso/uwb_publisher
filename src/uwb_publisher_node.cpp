#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp> // Corrected header for Boost.Bind

using namespace boost::asio;
using namespace boost::placeholders;

class UWBPublisher : public rclcpp::Node {
public:
    UWBPublisher() : Node("uwb_publisher"), io(), serial(io) {
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        // Set up the serial port.
        serial.open(serial_port);
        serial.set_option(serial_port_base::baud_rate(baud_rate));

        publisher_ = this->create_publisher<std_msgs::msg::String>("uwb_data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&UWBPublisher::publishData, this));
    }

private:
    void publishData() {
        if (serial.is_open()) {
            std_msgs::msg::String msg;
            std::string read_data;
            // Properly read from serial without 'available'
            boost::asio::streambuf buf;
            boost::asio::read_until(serial, buf, '\n'); // Assuming your data ends with newline
            std::istream is(&buf);
            std::getline(is, read_data);
            msg.data = read_data;
            publisher_->publish(msg);
        }
    }

    io_service io;
    serial_port serial;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBPublisher>());
    rclcpp::shutdown();
    return 0;
}