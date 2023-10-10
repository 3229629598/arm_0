#ifndef pick_task_hpp
#define pick_task_hpp

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace pick_task_ns
{
    namespace mtc = moveit::task_constructor;

    class MTCTaskNode
    {
        public:
        MTCTaskNode(const rclcpp::NodeOptions& options);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

        void doTask();

        void setupPlanningScene();

        private:
        // Compose an MTC task from a series of stages.
        mtc::Task createTask();
        mtc::Task task_;
        rclcpp::Node::SharedPtr node_;
    };
}

#endif
