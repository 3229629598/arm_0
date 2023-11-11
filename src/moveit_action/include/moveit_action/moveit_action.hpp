#ifndef moveit_action_hpp
#define moveit_action_hpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

namespace moveit_action_ns
{
    using namespace std::placeholders;
    using FollowJointTrajectory=control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory=rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    class Moveit_Action_Node : public rclcpp::Node
    {
        public:
        Moveit_Action_Node();
        ~Moveit_Action_Node();
        private:
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
        void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
        trajectory_msgs::msg::JointTrajectory jt_data;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jtp_pub;
        sensor_msgs::msg::JointState js_data;
    };

}

#endif