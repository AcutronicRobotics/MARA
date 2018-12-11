# MARA

This repository provides Gazebo ROS 2.0 support for [MARA](https://acutronicrobotics.com/products/mara/).

![](https://acutronicrobotics.com/docs/user/pages/02.Products/01.MARA/MARA2.jpg)

## Packages

 - `mara_bringup`: roslaunch scripts for starting the MARA.
 - `mara_description`: 3D models of the MARA for simulation and visualization.
 - `mara_gazebo`: Gazebo simulation package for the MARA.
 - `mara_gazebo_plugins`: MARA Gazebo plugins for sensors and motors.
 - `robotiq_140_gripper_description`: 3D models of the Robotiq 140 gripper for simulation and visualization.
 - `robotiq_140_gripper_gazebo_plugins`: Robotiq 140 gripper Gazebo plugins for the gripper.
 - `mara_utils_scripts`: Some scripts to move the MARA or spawn the model.

## Dependencies

 - [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) branch: `ros2`.
 - [HRIM](https://github.com/erlerobot/HRIM/).
 - [control_msgs](https://github.com/erlerobot/control_msgs) branch: `ardent`.
 - [image_common](https://github.com/ros-perception/image_common) branch: `ros2`.

## Example code

 - [mara_examples](https://github.com/AcutronicRobotics/mara_examples.git)

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

## Usage with Gazebo Simulation

There are launch files available to bringup the MARA robot

Don't forget to source the correct setup shell files and use a new terminal for each command!

### Terminal 1:

To bring up the simulated robot in Gazebo:

```
source ~/ros2_mara_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/mara
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/mara/mara_gazebo_plugins/build/
gazebo --verbose -s libgazebo_ros_factory.so
```

### Terminal 2:

Spawing the model:

```
source ~/ros2_mara_ws/install/setup.bash
cd ~/ros2_mara_ws/src/mara/mara_utils_scripts
python3 spawn_entity.py
```

Publishing robot model

```
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_bringup mara_bringup.launch.py
```

### Terminal 3

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
