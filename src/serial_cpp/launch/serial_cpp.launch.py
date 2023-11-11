import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

pkg_share=get_package_share_directory('serial_cpp')
config_name = "serial_config.yaml"

config_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share,"config","serial_config.yaml")
    )

def generate_launch_description():
    serial_cpp = Node(
        package="serial_cpp",
        executable="serial_cpp_node",
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        config_params,
        serial_cpp
    ])
