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
 - [control_msgs](https://github.com/ros-controls/control_msgs) branch: `bouncy-devel`.
 - [image_common](https://github.com/ros-perception/image_common) branch: `ros2`.
 - [vision_opencv](https://github.com/ros-perception/vision_opencv) branch: `ros2`
 - sudo apt install python3-numpy

## Example code

 - [mara_examples](https://github.com/AcutronicRobotics/mara_examples.git)

## Install

### Install ROS 2.0

Install ROS 2.0 following the official instructions: [source](https://index.ros.org/doc/ros2/Linux-Development-Setup/) [debian packages](https://index.ros.org/doc/ros2/Linux-Install-Debians/).

## Create mara ROS 2.0 workspace
Create a ROS workspace, for example:

```bash
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA -b ros2
git clone https://github.com/ros-simulation/gazebo_ros_pkgs -b ros2
git clone https://github.com/erlerobot/HRIM/
git clone https://github.com/ros-controls/control_msg -b bouncy-devel
git clone https://github.com/ros-perception/image_common -b ros2
git clone https://github.com/ros-perception/vision_opencv -b ros2
sudo apt install python3-numpy
```

## Compile

**Optional note**: If you want to use MoveIT! you need to source ROS 1.0 environment variables. Typically, if you have installed ROS `Kinetic`, you need to source the following file:

```bash
source /opt/ros/kinetic/setup.bash
```

Right now you can compile the code:

```bash
cd ~/ros2_mara_ws && colcon build --merge-install  
```

### MoveIT!

```bash
mkdir -p ~/ros_mara_ws/src
cd ~/ros_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA_ROS1
cd ~/ros_mara_ws/
catkin_make_isolated --install
```

## Usage with Gazebo Simulation

There are launch files available to bringup the MARA robot

Don't forget to source the correct setup shell files and use a new terminal for each command!

### Terminal 1:

To bring up the simulated robot in Gazebo:

```
source ~/ros2_mara_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/MARA
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/MARA/mara_gazebo_plugins/build/
gazebo --verbose -s libgazebo_ros_factory.so
```

### Terminal 2:

Spawing the model:

```
source ~/ros2_mara_ws/install/setup.bash
ros2 run mara_utils_scripts spawn_entity.py
```

Publishing robot model

```
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_bringup mara_bringup.launch.py
```

### RVIZ2

To visualize the robot with RVIZ2, you should type the following instructions:

```
source ~/ros2_mara_ws/install/setup.bash
rviz2
```

### MoveIT!

MoveIT! is not yet available for ROS 2.0. For now, if you want to use it with MARA we need to launch a set of nodes to create a bridge between ROS and ROS 2.0. It's quite complex to configure, please be pacient.

#### ROS 2.0

We need to launch in ROS 2.0 two nodes. One is the bridge between ROS and ROS 2.0 and the other one is the node that fetches all the state from the motors and create a topic called `/mara_controller/state`.

##### Terminal 1:

We need to run this node to create a bridge bewteen ROS and ROS 2.0. The topics that will be available are `/mara_controller/state`, `/joints_state` and `hros_actuation_servomotor_*********/trajectory`. Type the following command to run the bridge:

```
source /opt/ros/kinetic/setup.bash
source ~/ros2_mara_ws/install/setup.bash
ros2 run individual_trajectories_bridge individual_trajectories_bridge -motors `ros2 pkg prefix individual_trajectories_bridge`/share/individual_trajectories_bridge/motors.yaml
```

##### Terminal 2:

This ROS 2.0 node will fetch all the `hros_actuation_servomotor_*********/state` topics define in the config file and these topics data will be republish in a topic called `/mara_controller/state`. This node also will be subscribe to `/mara_controller/command` and it will republish the data in to the corresponding H-ROS topic `hros_actuation_servomotor_*********/goal`. To run this ROS 2.0 just type:

```
source ~/ros2_mara_ws/install/setup.bash
ros2 run hros_cognition_mara_components hros_cognition_mara_components -motors `ros2 pkg prefix hros_cognition_mara_components`/share/hros_cognition_mara_components/link_order.yaml
```

#### ROS

Right now we need the nodes to run MoveIT!. We should execute the following nodes for setting up the `robot_description` parameter, a node that handles `follow_joint_trajectory` topic, the MoveIT nodes and finally RVIZ to move the robot.

#### Terminal 1:

This node will set the `robot_description` parameter. MoveIT make use of this parameter to calculate the forward and inverse kinematics.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
roslaunch mara_bringup mara_bringup.launch
```
#### Terminal 2:

This node will handle the `follow_joint_trajectory` topic. This node is subscribed to this topic and it will republish the data in the corresponding `hros_actuation_servomotor_*********/trajectory`.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
rosrun mara_bringup follow_joints_individual_trajectory.py
```

#### Terminal 3:

This terminal will launch MoveIT.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
roslaunch mara_moveit_config mara_moveit_planning_execution.launch
```

#### Terminal 4:

If you want to use RVIZ to move the robot that we need to type the following instructions.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
roslaunch mara_moveit_config moveit_rviz.launch config:=true
```

## Others

Convert URDF into sdf

```
xacro --inorder /home/erle/ros2_mara_ws/src/MARA/mara_description/urdf/mara_robot_camera_top.urdf.xacro -o /home/erle/ros2_mara_ws/src/MARA/mara_description/urdf/mara_robot_camera_top.urdf
gz sdf -p /home/erle/ros2_mara_ws/src/MARA/mara_description/urdf/mara_robot_camera_top.urdf > mara_robot_camera_top.sdf
```
