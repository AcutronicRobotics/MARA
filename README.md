# modular articulated arm - mara

## Install

### Install ROS 2.0

Install ROS 2.0 following the official instructions: [source](https://index.ros.org/doc/ros2/Linux-Development-Setup/) [debian packages](https://index.ros.org/doc/ros2/Linux-Install-Debians/).

## Create mara ROS 2.0 workspace
Create a ROS workspace, for example:

```
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws/src
git clone https://github.com/erlerobot/mara -b ros2
```

## Compile

```
cd ~/ros2_mara_ws && colcon build --merge-install  
```

## Launch

Terminal 1:

```
source ~/ros2_mara_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/mara
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/mara/mara_gazebo_plugins/build/'
gazebo --verbose ~/ros2_mara_ws/src/mara/mara_description/urdf/mara_robot_camera_top.urdf -s libgazebo_ros_init.so
```
or
```
source ~/ros2_mara_ws/install/setup.bash
source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/mara
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/mara/mara_gazebo_plugins/build/'
gazebo --verbose -s libgazebo_ros_factory.so
```
this option needs to spawn the model
```
source ~/ros2_mara_ws/install/setup.bash
cd ~/ros2_mara_ws/src/mara/mara_utils_scripts
python3 spawn_entity.py
```

Terminal 2:

```
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_bringup mara_bringup.launch.py
```

Terminal 3

```
source ~/ros2_mara_ws/install/setup.bash
rviz2
```

## Others

Convert URDF into sdf

```
xacro --inorder /home/erle/ros2_mara_ws/src/mara/mara_description/urdf/mara_robot_camera_top.urdf.xacro -o /home/erle/ros2_mara_ws/src/mara/mara_description/urdf/mara_robot_camera_top.urdf
gz sdf -p /home/erle/ros2_mara_ws/src/mara/mara_description/urdf/mara_robot_camera_top.urdf > mara_robot_camera_top.sdf
```
