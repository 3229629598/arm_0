#ifndef serial_hpp
#define serial_hpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <serial_cpp/serial_driver.hpp>
#include <msg_converters/converters.hpp>

namespace serial_cpp_ns
{
    using namespace drivers::serial_driver;
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    class Serial_cpp : public rclcpp::Node
    {
        public:
        Serial_cpp();
        ~Serial_cpp();

        private:
        
        std::unique_ptr<IoContext> io_ctx{};
        std::string device_name{};
        std::unique_ptr<SerialPortConfig> serial_config;
        std::unique_ptr<SerialDriver> serial_driver;

        void get_params();        

        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_pub;
        void rx_callback(const std::vector<uint8_t> & rx_buf, const size_t & bytes_transferred);
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub;
        void tx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr tx_buf);

        rclcpp::TimerBase::SharedPtr timer;
        void serial_self_test();
    };
}

#endif
