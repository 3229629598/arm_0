from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_ctrl = Node(
        package="moveit_ctrl",
        executable="moveit_ctrl_node"
    )
    launch_description = LaunchDescription(
        [moveit_ctrl])
    return launch_description