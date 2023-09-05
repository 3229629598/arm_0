#include <moveit_ctrl/moveit_ctrl.hpp>

namespace moveit_ctrl_ns
{
    static const std::string PLANNING_GROUP = "arm";

    Moveit_Ctrl::Moveit_Ctrl(const rclcpp::NodeOptions & options):Node("moveit_ctrl_node",options)
    {
        RCLCPP_INFO(get_logger(),"Moveit_ctrl_node is running.");
        request_sub=this->create_subscription<std_msgs::msg::UInt8>("/arm_request",10,std::bind(&Moveit_Ctrl::request_callback, this, std::placeholders::_1));
        move_group_ptr.reset(new moveit::planning_interface::MoveGroupInterface(nh,PLANNING_GROUP));
        joint_model_group=move_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    }

    Moveit_Ctrl::~Moveit_Ctrl()
    {}

    void Moveit_Ctrl::request_callback(const std_msgs::msg::UInt8::SharedPtr request_msg)
    {

    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<moveit_ctrl_ns::Moveit_Ctrl>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}