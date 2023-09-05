#ifndef moveit_ctrl_hpp
#define moveit_ctrl_hpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

namespace moveit_ctrl_ns
{
    class Moveit_Ctrl : public rclcpp::Node
    {
        public:
        Moveit_Ctrl(const rclcpp::NodeOptions & options);
        ~Moveit_Ctrl();
        private:
        std::shared_ptr<rclcpp::Node> nh;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr request_sub;
        void request_callback(const std_msgs::msg::UInt8::SharedPtr request_msg);
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_ptr;
        std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_ptr;
        const moveit::core::JointModelGroup* joint_model_group;
    };
}

#endif