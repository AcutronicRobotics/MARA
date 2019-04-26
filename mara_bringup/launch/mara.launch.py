import sys
import os
import launch

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('mara_description'), 'urdf', 'mara_robot_gripper_140.urdf')
    install_dir = get_package_prefix('mara_gazebo_plugins')

    ld = LaunchDescription([
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='hros_cognition_mara_components', node_executable='hros_cognition_mara_components', output='screen',
            arguments=["-motors", install_dir + "/share/hros_cognition_mara_components/link_order.yaml"])
    ])
    return ld
