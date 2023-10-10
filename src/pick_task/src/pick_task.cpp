#include <pick_task/pick_task.hpp>

namespace pick_task_ns
{
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pick_task_node is running");

    MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options) : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
    {
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
    {
        return node_->get_node_base_interface();
    }

    void MTCTaskNode::setupPlanningScene()
    {
        moveit_msgs::msg::CollisionObject object;
        object.id = "object";
        object.header.frame_id = "world";
        object.primitives.resize(1);
        object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        object.primitives[0].dimensions = { 0.1, 0.02 };

        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = -0.25;
        object.pose = pose;

        moveit::planning_interface::PlanningSceneInterface psi;
        psi.applyCollisionObject(object);
    }

    void MTCTaskNode::doTask()
    {
        task_ = createTask();

        try
        {
            task_.init();
        }
        catch (mtc::InitStageException& e)
        {
            RCLCPP_ERROR_STREAM(LOGGER, e);
            return;
        }

        if (!task_.plan(5))
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
            return;
        }
        task_.introspection().publishSolution(*task_.solutions().front());

        auto result = task_.execute(*task_.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
            return;
        }

        return;
    }

    mtc::Task MTCTaskNode::createTask()
    {
        mtc::Task task;
        task.stages()->setName("demo task");
        task.loadRobotModel(node_);

        const auto& arm_group_name = "arm";
        const auto& hand_group_name = "hand";
        const auto& hand_frame = "ee_link";

        // Set task properties
        task.setProperty("group", arm_group_name);
        task.setProperty("eef", hand_group_name);
        task.setProperty("ik_frame", hand_frame);

        // Disable warnings for this line, as it's a variable that's set but not used in this example
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
        mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
        #pragma GCC diagnostic pop

        auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
        current_state_ptr = stage_state_current.get();
        task.add(std::move(stage_state_current));

        auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
        auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScaling(1.0);
        cartesian_planner->setMaxAccelerationScaling(1.0);
        cartesian_planner->setStepSize(.01);

        // clang-format off
        auto stage_open_hand =
            std::make_unique<mtc::stages::MoveTo>("hand_open", interpolation_planner);
        // clang-format on
        stage_open_hand->setGroup(hand_group_name);
        stage_open_hand->setGoal("open");
        task.add(std::move(stage_open_hand));

        return task;
    }

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<pick_task_ns::MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node->getNodeBaseInterface());
        executor.spin();
        executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}