#ifndef moveit_ctrl_hpp
#define moveit_ctrl_hpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
        Moveit_Ctrl();
        ~Moveit_Ctrl();

        private:
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr request_sub;
        void request_callback(const std_msgs::msg::UInt8::SharedPtr request_msg);
        std::vector<std::vector<double>> joint_goal;
        uint8_t joint_goal_num;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
        geometry_msgs::msg::Pose previous_target_pose;
        std::vector<geometry_msgs::msg::Pose> pose_goal;

        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::PlanningSceneInterface planning_scene;

        moveit_msgs::msg::CollisionObject collision_object,object_to_attach;
        shape_msgs::msg::SolidPrimitive primitive;
        geometry_msgs::msg::Pose ball1_pose,ball2_pose;
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        std::vector<std::string> touch_links;
    };
}

#endif