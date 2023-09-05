import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
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

move_group_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("arm_0_moveit_config"),"launch","move_group.launch.py")
    )
)

rsp_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("arm_0_moveit_config"),"launch","rsp.launch.py")
    )
)

moveit_rviz=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("arm_0_moveit_config"),"launch","moveit_rviz.launch.py")
    ),
    condition=IfCondition(use_rviz)
)

moveit_ctrl_launch=IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    	os.path.join(get_package_share_directory("moveit_ctrl"),"launch","moveit_ctrl.launch.py")
    )
)

ld.add_action(serial_py_launch)
ld.add_action(data_process_launch)
ld.add_action(move_group_launch)
ld.add_action(rsp_launch)
ld.add_action(moveit_rviz)
ld.add_action(moveit_ctrl_launch)

def generate_launch_description():
    return ld
