from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_action = Node(
        package="moveit_action",
        executable="moveit_action_node"
    )
    launch_description = LaunchDescription(
        [moveit_action])
    return launch_description