import sys
import os
import launch

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('mara_description'), 'urdf', 'mara_robot_gripper_140_camera.urdf')
    mara = get_package_share_directory('mara_gazebo_plugins')
    install_dir = get_package_prefix('mara_gazebo_plugins')
    print("plugins", install_dir)
    print("mara", mara)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    try:
        envs = {}
        for key in os.environ.__dict__["_data"]:
            key = key.decode("utf-8")
            if (key.isupper()):
                envs[key] = os.environ[key]
    except Exception as e:
        print("Error with Envs: " + str(e))
        return None

    ld = LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen',
            env=envs
        ),
        Node(package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', arguments=[urdf]),
        Node(package='mara_utils_scripts', node_executable='spawn_mara_gripper_140_camera.py', output='screen'),
        Node(package='hros_cognition_mara_components', node_executable='hros_cognition_mara_components', output='screen',
            arguments=["-motors", install_dir + "/share/hros_cognition_mara_components/link_order.yaml"])
    ])
    return ld
