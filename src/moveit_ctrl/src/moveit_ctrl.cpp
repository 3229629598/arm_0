#include <moveit_ctrl/moveit_ctrl.hpp>

namespace moveit_ctrl_ns
{
    const std::string PLANNING_GROUP = "arm";

    Moveit_Ctrl::Moveit_Ctrl():Node("moveit_ctrl_node"),move_group(std::shared_ptr<rclcpp::Node>(std::move(this)),PLANNING_GROUP)
    {
        RCLCPP_INFO(get_logger(),"Moveit_ctrl_node is running.");

        this->move_group.setMaxAccelerationScalingFactor(1);
        this->move_group.setMaxVelocityScalingFactor(1);
        this->move_group.setGoalJointTolerance(0.001);

        request_sub=this->create_subscription<std_msgs::msg::UInt8>("/arm_request",10,std::bind(&Moveit_Ctrl::request_callback, this, std::placeholders::_1));
        pose_sub=this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",10,std::bind(&Moveit_Ctrl::pose_callback, this, std::placeholders::_1));

        joint_goal.push_back({0.0,0.0,0.0,0.0,0.0,0.0});
        joint_goal.push_back({0.1,0.0,0.0,0.0,0.0,0.0});
        joint_goal.push_back({0.2,0.0,0.0,0.0,0.0,0.0});
        joint_goal.push_back({0.2,0.5,0.0,1.5708,0.0,1.5708});
        joint_goal_num=joint_goal.size();
    }

    Moveit_Ctrl::~Moveit_Ctrl()
    {}

    void Moveit_Ctrl::request_callback(const std_msgs::msg::UInt8::SharedPtr request_msg)
    {
        if(request_msg->data<joint_goal_num)
        {
            this->move_group.setJointValueTarget(joint_goal[request_msg->data]);
            bool success=(this->move_group.move()==moveit::core::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(get_logger(), "Arm request %s", success ? "SUCCESS" : "FAILED");
        }
    }

    void Moveit_Ctrl::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        this->move_group.setPoseTarget(pose_msg->pose);
        bool success=(this->move_group.move()==moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(get_logger(), "Target pose %s", success ? "SUCCESS" : "FAILED");
        previous_target_pose=pose_msg->pose;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<moveit_ctrl_ns::Moveit_Ctrl>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
