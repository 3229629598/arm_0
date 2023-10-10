import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder("moveit_resources_panda")
    #     .planning_pipelines(pipelines=["ompl"])
    #     .robot_description(file_path="config/panda.urdf.xacro")
    #     .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
    #     .to_moveit_configs()
    # )
    moveit_config = MoveItConfigsBuilder("engineer2023", package_name="engineer2023_moveit_config").to_moveit_configs()
    pick_task_node = Node(
        package="pick_task",
        executable="pick_task_node",
        output="screen",
        parameters=[
            # os.path.join(
            #     get_package_share_directory("pick_task"),
            #     "config",
            #     "pick_config.yaml",
            # ),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([pick_task_node])
