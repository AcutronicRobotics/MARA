import sys
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    args = sys.argv[1:]
    configFile = "/share/hros_cognition_mara_components/motors.yaml"
    if "--urdf" in args:
        urdfName = args[args.index("--urdf")+1]
        if("train" in urdfName) or ("run" in urdfName):
            urdfName = "reinforcement_learning/" + urdfName
        if("two" in urdfName):
            configFile = "/share/hros_cognition_mara_components/two_motors.yaml"
    else:
        urdfName = 'mara_robot'

    urdf = os.path.join(get_package_share_directory('mara_description'), 'urdf/', urdfName + '.urdf')
    assert os.path.exists(urdf)

    install_dir = get_package_prefix('mara_gazebo_plugins')

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
        Node(package='mara_utils_scripts', node_executable='spawn_mara.py', arguments=[urdf], output='screen'),
        Node(package='hros_cognition_mara_components', node_executable='hros_cognition_mara_components', output='screen', arguments=["-motors", install_dir + configFile, "sim"])
    ])
    return ld
