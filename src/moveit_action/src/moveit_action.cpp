#include <moveit_action/moveit_action.hpp>

namespace moveit_action_ns
{
    Moveit_Action_Node::Moveit_Action_Node():Node("moveit_action_node")
    {
        RCLCPP_INFO(get_logger(),"Moveit_action_node is running.");
        this->action_server=rclcpp_action::create_server<FollowJointTrajectory>(
        this, "/arm_controller/follow_joint_trajectory",
        std::bind(&Moveit_Action_Node::handle_goal, this, _1, _2),
        std::bind(&Moveit_Action_Node::handle_cancel, this, _1),
        std::bind(&Moveit_Action_Node::handle_accepted, this, _1)
        );
        jtp_pub=this->create_publisher<sensor_msgs::msg::JointState>("/jtp_data",10);
        js_data.name.insert(js_data.name.begin(), {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    };

    Moveit_Action_Node::~Moveit_Action_Node()
    {}

    rclcpp_action::GoalResponse Moveit_Action_Node::handle_goal(const rclcpp_action::GoalUUID & uuid,std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Get a joint trajectory.");
        (void)uuid;
        jt_data=goal->trajectory;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse Moveit_Action_Node::handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void Moveit_Action_Node::execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        (void)goal_handle;
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        auto point_num=jt_data.points.size();
        int64_t last_time_ns=jt_data.points[0].time_from_start.sec*1e9+jt_data.points[0].time_from_start.nanosec;
        uint16_t ctrl_frequence=200;

        for(int i=1;i<point_num;i++)
        {
            if(goal_handle->is_canceling())
            {
                goal_handle->canceled(result);
                return;
            }

            int64_t this_time_ns=jt_data.points[i].time_from_start.sec*1e9+jt_data.points[i].time_from_start.nanosec;
            uint64_t point_sum=(this_time_ns-last_time_ns)*ctrl_frequence/1e9;
            int64_t sleep_time_ns=1e9/ctrl_frequence;

            js_data.position.assign(jt_data.points[i-1].positions.begin(),jt_data.points[i-1].positions.end());
            for(int j=0;j<point_sum;j++)
            {
                for(int k=0;k<jt_data.points[i].positions.size();k++)
                    js_data.position[k]+=(jt_data.points[i].positions[k]-jt_data.points[i-1].positions[k])/point_sum;
                js_data.header.stamp=this->now();
                jtp_pub->publish(js_data);
                rclcpp::sleep_for(std::chrono::nanoseconds(sleep_time_ns));
            }
            
            last_time_ns=this_time_ns;
        }
        goal_handle->succeed(result);
    }

    void Moveit_Action_Node::handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        std::thread{std::bind(&Moveit_Action_Node::execute, this, _1), goal_handle}.detach();
    }

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<moveit_action_ns::Moveit_Action_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}