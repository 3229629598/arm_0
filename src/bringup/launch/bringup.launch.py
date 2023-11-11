import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

ld=LaunchDescription()

fake_test=LaunchConfiguration('fake_test', default='false')
use_rviz=LaunchConfiguration('use_rviz', default='true')

serial_py_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("serial_py"),"launch","serial_py.launch.py")
    ),
    condition=UnlessCondition(fake_test)
)

data_process_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("data_process"),"launch","data_process.launch.py")
    )
)

config_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("bringup"),"launch","config.launch.py")
    ),
    launch_arguments={'fake_test': fake_test,
                      'use_rviz' : use_rviz}.items()
)

moveit_action_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("moveit_action"),"launch","moveit_action.launch.py")
    )
)

moveit_ctrl_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("moveit_ctrl"),"launch","moveit_ctrl.launch.py")
    )
)

static_tf_node = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher",
    output="log",
    arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
)

ld.add_action(serial_py_launch)
ld.add_action(data_process_launch)
ld.add_action(config_launch)
ld.add_action(moveit_action_launch)
ld.add_action(moveit_ctrl_launch)
ld.add_action(static_tf_node)

def generate_launch_description():
    return ld
