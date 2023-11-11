from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    data_process = Node(
        package="data_process",
        executable="data_process_node"
    )
    launch_description = LaunchDescription(
        [data_process])
    return launch_description