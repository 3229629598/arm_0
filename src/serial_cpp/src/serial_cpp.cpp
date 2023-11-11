#include <serial_cpp/serial_cpp.hpp>

namespace serial_cpp_ns
{
    Serial_cpp::Serial_cpp():Node("serial_cpp_node"),io_ctx{new IoContext(2)},serial_driver(new SerialDriver(*io_ctx))
    {
        RCLCPP_INFO(get_logger(),"Serial_cpp_node is running.");
        get_params();
        rx_pub=this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rx_data",10);
        tx_sub=this->create_subscription<std_msgs::msg::UInt8MultiArray>("/tx_data",10,std::bind(&Serial_cpp::tx_callback,this,_1));
        timer=this->create_wall_timer(50ms, std::bind(&Serial_cpp::serial_self_test, this));
        while(1)
        {
            try
            {
                serial_driver->init_port(device_name, *serial_config);
                if (!serial_driver->port()->is_open())
                {
                    serial_driver->port()->open();
                    serial_driver->port()->async_receive(std::bind(&Serial_cpp::rx_callback,this,_1,_2));
                }
                break;
            }
            catch (const std::exception & ex)
            {
                RCLCPP_ERROR(get_logger(),"Error creating serial port: %s - %s",device_name.c_str(),ex.what());
            }
        };
        RCLCPP_INFO(get_logger(),"Success creating serial port: %s",device_name.c_str());
    }

    Serial_cpp::~Serial_cpp()
    {
        if (io_ctx)
            io_ctx->waitForExit();
    }

    void Serial_cpp::get_params()
    {
        uint32_t baud_rate{};
        auto fc = FlowControl::NONE;
        auto pt = Parity::NONE;
        auto sb = StopBits::ONE;

        try {
            device_name = declare_parameter<std::string>("device_name", "");
        } catch (rclcpp::ParameterTypeException & ex) {
            RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
            throw ex;
        }

        try {
            baud_rate = declare_parameter<int>("baud_rate", 0);
        } catch (rclcpp::ParameterTypeException & ex) {
            RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
            throw ex;
        }

        try {
            const auto fc_string = declare_parameter<std::string>("flow_control", "");

            if (fc_string == "none") {
            fc = FlowControl::NONE;
            } else if (fc_string == "hardware") {
            fc = FlowControl::HARDWARE;
            } else if (fc_string == "software") {
            fc = FlowControl::SOFTWARE;
            } else {
            throw std::invalid_argument{
                    "The flow_control parameter must be one of: none, software, or hardware."};
            }
        } catch (rclcpp::ParameterTypeException & ex) {
            RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
            throw ex;
        }

        try {
            const auto pt_string = declare_parameter<std::string>("parity", "");

            if (pt_string == "none") {
            pt = Parity::NONE;
            } else if (pt_string == "odd") {
            pt = Parity::ODD;
            } else if (pt_string == "even") {
            pt = Parity::EVEN;
            } else {
            throw std::invalid_argument{
                    "The parity parameter must be one of: none, odd, or even."};
            }
        } catch (rclcpp::ParameterTypeException & ex) {
            RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
            throw ex;
        }

        try {
            const auto sb_string = declare_parameter<std::string>("stop_bits", "");

            if (sb_string == "1" || sb_string == "1.0") {
            sb = StopBits::ONE;
            } else if (sb_string == "1.5") {
            sb = StopBits::ONE_POINT_FIVE;
            } else if (sb_string == "2" || sb_string == "2.0") {
            sb = StopBits::TWO;
            } else {
            throw std::invalid_argument{
                    "The stop_bits parameter must be one of: 1, 1.5, or 2."};
            }
        } catch (rclcpp::ParameterTypeException & ex) {
            RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
            throw ex;
        }

        serial_config = std::make_unique<SerialPortConfig>(baud_rate, fc, pt, sb);
    }    

    void Serial_cpp::rx_callback(const std::vector<uint8_t> &rx_buf,const size_t &bytes_transferred)
    {
        std_msgs::msg::UInt8MultiArray rx_data;
        drivers::common::to_msg(rx_buf,rx_data,bytes_transferred);
        rx_pub->publish(rx_data);
    }

    void Serial_cpp::tx_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr tx_data)
    {
        std::vector<uint8_t> tx_buf;
        drivers::common::from_msg(tx_data,tx_buf);
        serial_driver->port()->async_send(tx_buf);
    }

    void Serial_cpp::serial_self_test()
    {
        if(serial_driver->port()->is_break)
        {
            try
            {
                serial_driver->port()->close();
                serial_driver->port()->open();
                serial_driver->port()->async_receive(std::bind(&Serial_cpp::rx_callback,this,_1,_2));
                RCLCPP_INFO(get_logger(),"Trying to reconnect");
            }
            catch(const std::exception& e)
            {}
        }      
    }

}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_cpp_ns::Serial_cpp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}