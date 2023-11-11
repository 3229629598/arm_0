from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    serial_py = Node(
        package="serial_py",
        executable="serial_py_node"
    )
    launch_description = LaunchDescription(
        [serial_py])
    return launch_description
