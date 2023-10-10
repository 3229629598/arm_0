#include <moveit_ctrl/moveit_ctrl.hpp>

namespace moveit_ctrl_ns
{
    const std::string PLANNING_GROUP = "arm";

    Moveit_Ctrl::Moveit_Ctrl():Node("moveit_ctrl_node"),move_group(std::shared_ptr<rclcpp::Node>(std::move(this)),PLANNING_GROUP)
    {
        RCLCPP_INFO(get_logger(),"Moveit_ctrl_node is running.");

        this->move_group.setMaxAccelerationScalingFactor(1);
        this->move_group.setMaxVelocityScalingFactor(1);

        request_sub=this->create_subscription<std_msgs::msg::UInt8>("/arm_request",10,std::bind(&Moveit_Ctrl::request_callback, this, std::placeholders::_1));
        pose_sub=this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",10,std::bind(&Moveit_Ctrl::pose_callback, this, std::placeholders::_1));

        joint_goal.push_back({0.3,0.0,0.25,0.09,3.1416,0.0,0.0});
        joint_goal.push_back({0.2,0.3,0.0,0.09,0.0,0.0,-1.5708});
        joint_goal_num=joint_goal.size();

        collision_object.header.frame_id = move_group.getPlanningFrame();
        collision_object.id = "ball1";
        object_to_attach.header.frame_id = move_group.getEndEffectorLink();
        object_to_attach.id = "ball2";

        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[primitive.SPHERE_RADIUS] = 0.05;
        
        ball1_pose.orientation.w = 1.0;
        ball1_pose.position.x = 0.0;
        ball1_pose.position.y = 0.7;
        ball1_pose.position.z = 0.8;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(ball1_pose);
        collision_object.operation = collision_object.ADD;
        collision_objects.push_back(collision_object);

        ball2_pose.orientation.w = 1.0;
        ball2_pose.position.z = 0.05;

        object_to_attach.primitives.push_back(primitive);
        object_to_attach.primitive_poses.push_back(ball2_pose);
        object_to_attach.operation = object_to_attach.ADD;

        touch_links.push_back("base_link");
        touch_links.push_back("link1");
        touch_links.push_back("link2");
        touch_links.push_back("link3");
        touch_links.push_back("link4");
        touch_links.push_back("link5");
        touch_links.push_back("link6");
        touch_links.push_back("link7");
        touch_links.push_back("ee_link");
        touch_links.push_back("link8");
        touch_links.push_back("link9");
        touch_links.push_back("link10");
        touch_links.push_back("link11");
    }

    Moveit_Ctrl::~Moveit_Ctrl()
    {}

    void Moveit_Ctrl::request_callback(const std_msgs::msg::UInt8::SharedPtr request_msg)
    {
        bool success;
        if(request_msg->data==1)
        {
            move_group.detachObject(object_to_attach.id);
            std::vector<std::string> object_ids;
            object_ids.push_back(object_to_attach.id);
            planning_scene.removeCollisionObjects(object_ids);
            planning_scene.addCollisionObjects(collision_objects);

            geometry_msgs::msg::Pose goal_pose=ball1_pose;
            goal_pose.position.z-=0.05;
            this->move_group.setPoseTarget(goal_pose);
            success=(this->move_group.move()==moveit::core::MoveItErrorCode::SUCCESS);
            RCLCPP_INFO(get_logger(), "Arm request %s", success ? "SUCCESS" : "FAILED");
        }
        else
        {
            std::vector<std::string> object_ids;
            object_ids.push_back(collision_object.id);
            planning_scene.removeCollisionObjects(object_ids);
            planning_scene.applyCollisionObject(object_to_attach);
            move_group.attachObject(object_to_attach.id, "ee_link", touch_links);

            this->move_group.setJointValueTarget(joint_goal[request_msg->data]);
            success=(this->move_group.move()==moveit::core::MoveItErrorCode::SUCCESS);
            
        }
        RCLCPP_INFO(get_logger(), "Arm request %s", success ? "SUCCESS" : "FAILED");
        if(!success)
        {
            std::vector<std::string> object_ids;
            object_ids.push_back(collision_object.id);
            object_ids.push_back(object_to_attach.id);
            planning_scene.removeCollisionObjects(object_ids);
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
