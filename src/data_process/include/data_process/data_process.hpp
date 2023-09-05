#ifndef data_process_hpp
#define data_process_hpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <data_process/crc.hpp>
#include <data_process/packet.hpp>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

namespace data_process_ns
{
    class Data_Process : public rclcpp::Node
    {
        public:
        Data_Process(const rclcpp::NodeOptions & options);
        ~Data_Process();
        private:
        rclcpp::CallbackGroup::SharedPtr callback_group_sub;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub;
        void process_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr rx_buf);
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub;
        std_msgs::msg::UInt8MultiArray tx_buf;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_request_pub;
        std_msgs::msg::UInt8 arm_request_msg;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
        sensor_msgs::msg::JointState joint_state;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr jtp_sub;
        void jtp_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr jtp_data);
        uint8_t joint_num;
    };
}

#endif
