#include <data_process/data_process.hpp>

namespace data_process_ns
{
    Data_Process::Data_Process(const rclcpp::NodeOptions & options):Node("data_process_node",options)
    {
        RCLCPP_INFO(get_logger(),"Data_process_node is running.");
        callback_group_sub=this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.callback_group = callback_group_sub;
        rx_sub=this->create_subscription<std_msgs::msg::UInt8MultiArray>("/rx_data", 10, std::bind(&Data_Process::process_callback, this, std::placeholders::_1),sub_opt);
        tx_pub=this->create_publisher<std_msgs::msg::UInt8MultiArray>("/tx_data",10);
        joint_state_pub=this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
        arm_request_pub=this->create_publisher<std_msgs::msg::UInt8>("/arm_request",10);
        jtp_sub=this->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>("/jtp_data", 10, std::bind(&Data_Process::jtp_callback, this, std::placeholders::_1),sub_opt);
        
        joint_num=6;
        arm_request_msg.data=0xff;
        joint_state.position.resize(joint_num);
        joint_state.velocity.resize(joint_num);
        joint_state.effort.resize(joint_num);
        joint_state.header.frame_id = "base_link";
        joint_state.name.insert(joint_state.name.begin(), {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    }

    Data_Process::~Data_Process()
    {}

    void Data_Process::process_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr rx_buf)
    {
        rx_bag rx_data;
        std::copy(rx_buf->data.begin(), rx_buf->data.end(), reinterpret_cast<uint8_t *>(&rx_data));
        if(crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&rx_data),rx_len))
        {
            joint_state.header.stamp=this->now();
            for(int i=0;i<joint_num;i++)
                joint_state.position[i]=rx_data.joint_position[i];
            joint_state_pub->publish(joint_state);
            if(rx_data.arm_request!=arm_request_msg.data)
            {
                arm_request_msg.data=rx_data.arm_request;
                arm_request_pub->publish(arm_request_msg);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(),"crc false.");
        }
    }

    void Data_Process::jtp_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr jtp_data)
    {
        tx_bag tx_data;
        for(int i=0;i<joint_num;i++)
            tx_data.joint_goal[i]=jtp_data->positions[i];
        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&tx_data),tx_len);
        tx_buf.data=toVector(tx_data);
        tx_pub->publish(tx_buf);
    }

}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<data_process_ns::Data_Process>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}