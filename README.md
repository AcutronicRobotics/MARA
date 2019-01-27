# MARA

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/assets/images/AcutronicRobotics_logo_BlackBackground.png.pagespeed.ce.EDHWnZb3Nd.png" align="left" hspace="8" vspace="2" width="200"></a>

This is the official repository of [MARA](https://acutronicrobotics.com/products/mara/) modular robot, world's first modular cobot. MARA is the first robot which runs ROS 2.0 on each joint empowering new possibilities and applications in the professional and industrial landscapes of robotics. Built out of individual modules that natively run ROS 2.0, the modular robot arm can be physically extended in a seamless manner. MARA delivers industrial-grade features such as time synchronization or deterministic communication latencies.

Among other things, you will find in this repository instructions on how to simulate MARA with Gazebo and its integrations with ROS 2.0.

## Features

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/products/mara/images/xv2_MARA2-11.jpg.pagespeed.ic.QRaRP5N01r.webp" align="right" hspace="8" vspace="2" width="200"></a>

- **Powered by ROS 2.0**: a fully distributed software and hardware robotic architecture.

- **Highly customizable**: with daisy chaining, power and communication are exposed at the module level allowing for simplified extensions.

- **Real time data monitoring**: every H-ROS module is able to monitorize a variety of intrinsic aspects in real-time.

- **Power readings**: instantaneous voltage, current and power readings from each module, individually.

- **Automatic re-configuration**: embedded accelerometers, magnetometers and gyroscopes empower each module with inertial data.

- **HW and SW life cycle**: life cycle for each module allows greater control over the state of the ROS system and the underlying components.

- **Controllable from any ROS 2.0 enabled computer**: [ORC](https://acutronicrobotics.com/products/orc/) is the ideal complement for MARA, but not mandatory. Choose yourself how you steer MARA.

## Table of Contents
* [MARA](#mara)
    * [Features](#features)
    * [Table of Contents](#table-of-contents)
       * [Specifications](#specifications)
       * [Packages](#packages)
          * [Dependencies](#dependencies)
       * [Install](#install)
          * [Install ROS 2.0](#install-ros-20)
          * [Create mara ROS 2.0 workspace](#create-mara-ros-20-workspace)
          * [Compile](#compile)
          * [Set up MoveITt! (for now in ROS)](#set-up-moveitt-for-now-in-ros)
          * [Usage with Gazebo Simulation](#usage-with-gazebo-simulation)
             * [Terminal 1:](#terminal-1)
             * [Terminal 2:](#terminal-2)
             * [Rviz2](#rviz2)
             * [MoveIT!](#moveit)
             * [ROS 2.0](#ros-20)
                * [Terminal 1:](#terminal-1-1)
                * [Terminal 2:](#terminal-2-1)
             * [ROS](#ros)
                * [Terminal 1:](#terminal-1-2)
                * [Terminal 2:](#terminal-2-2)
                * [Terminal 3:](#terminal-3)
                * [Terminal 4:](#terminal-4)
       * [Others](#others)
       * [Example code](#example-code)
       * [Help](#help)

*Created by [gh-md-toc](https://github.com/ekalinin/github-markdown-toc)*

### Specifications

![](https://acutronicrobotics.com/products/mara/images/xMARA_evolution_end.jpg.pagespeed.ic.dVNwzZ6-4i.webp)


| Spec | Value |
|------|-------|
| Degrees of freedom | 6 DoF, extensible |
| Maximum speed | 90º/s |
| Repeatability | ±0.1 mm |
| Rated torque | 9.4/30 Nm |
| Payload | 3 Kg |
| Weight | 21 Kg |
| Height | 871 mm |
| Reach | 656 mm |
| Footprint | 204 mm |
| Robotics framework | ROS 2 Crystal Clemmys |
| Communication interfaces | 1 Gbps Ethernet, Compliant with TSN standards |
| Information model | Hardware Robot Information Model (HRIM®), version Anboto  |
| Security | Encrypted and secure computing environment, Secure data exchange capabilities |
| Automatic updates | Over-the-Air (OTA) |
| Datasheet | [Download datasheet](https://acutronicrobotics.com/products/mara/files/MARA_datasheet_v1.1.pdf) |

### Packages

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/products/mara/images/v2_MARA6_1-11.png" align="right" hspace="8" vspace="2" width="200"></a>

 - `mara_bringup`: roslaunch scripts for starting the MARA.
 - `mara_description`: 3D models of the MARA for simulation and visualization.
 - `mara_gazebo`: Gazebo simulation package for the MARA.
 - `mara_gazebo_plugins`: MARA Gazebo plugins for sensors and motors.
 - `robotiq_140_gripper_description`: 3D models of the Robotiq 140 gripper for simulation and visualization.
 - `robotiq_140_gripper_gazebo_plugins`: Robotiq 140 gripper Gazebo plugins for the gripper.
 - `mara_utils_scripts`: Some scripts to move the MARA or spawn the model.

#### Dependencies

 - [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) branch: `ros2`.
 - [HRIM](https://github.com/erlerobot/HRIM/).
 - [control_msgs](https://github.com/erlerobot/control_msgs) branch: `crystal`.
 - [image_common](https://github.com/ros-perception/image_common) branch: `ros2`.
 - [vision_opencv](https://github.com/ros-perception/vision_opencv) branch: `ros2`
 - sudo apt install python3-numpy


### Install

#### Install ROS 2.0

Install ROS 2.0 following the official instructions: [source](https://index.ros.org/doc/ros2/Linux-Development-Setup/) [debian packages](https://index.ros.org/doc/ros2/Linux-Install-Debians/).

#### Create mara ROS 2.0 workspace
Create a ROS workspace, for example:

```bash
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA
git clone https://github.com/ros-simulation/gazebo_ros_pkgs -b ros2
git clone https://github.com/erlerobot/HRIM/
git clone https://github.com/erlerobot/control_msgs -b crystal
git clone https://github.com/ros-perception/image_common -b ros2
git clone https://github.com/ros-perception/vision_opencv -b ros2
sudo apt install python3-numpy
```

Generate HRIM dependencies:

```bash
pip3 install lxml
cd ~/ros2_mara_ws/src/HRIM
python3 hrim.py generate models/actuator/servo/servo.xml
python3 hrim.py generate models/actuator/gripper/gripper.xml
```

#### Compile

**Optional note**: If you want to use MoveIT! you need to source ROS 1.0 environment variables. Typically, if you have installed ROS `Kinetic`, you need to source the following file:

```bash
source /opt/ros/kinetic/setup.bash
```

Right now you can compile the code:

```bash
source /opt/ros/crystal/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install
```

#### Set up MoveITt! (for now in ROS)

```bash
mkdir -p ~/ros_mara_ws/src
cd ~/ros_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA_ROS1
cd ~/ros_mara_ws/
catkin_make_isolated --install
```

#### Usage with Gazebo Simulation

There are launch files available to bringup the MARA robot with MoveIt! and Rviz2, altogether.

Don't forget to source the correct setup shell files and use a new terminal for each command!

##### Terminal 1:

To bring up the simulated robot in Gazebo:

```
source ~/ros2_mara_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_mara_ws/src/MARA
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_mara_ws/src/MARA/mara_gazebo_plugins/build/
gazebo --verbose -s libgazebo_ros_factory.so
```

##### Terminal 2:

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

##### Rviz2

To visualize the robot with Rviz2, you should type the following instructions:

```
source ~/ros2_mara_ws/install/setup.bash
rviz2
```

##### MoveIT!

MoveIT! is not yet available for ROS 2.0. For now, if you want to use it with MARA we need to launch a set of nodes to create a bridge between ROS and ROS 2.0. It's quite complex to configure, please be pacient.

##### ROS 2.0

We need to launch in ROS 2.0 two nodes. One is the bridge between ROS and ROS 2.0 and the other one is the node that fetches all the state from the motors and create a topic called `/mara_controller/state`.

###### Terminal 1:

We need to run this node to create a bridge bewteen ROS and ROS 2.0. The topics that will be available are `/mara_controller/state`, `/joints_state` and `hros_actuation_servomotor_*********/trajectory`. Type the following command to run the bridge:

```
source /opt/ros/kinetic/setup.bash
source ~/ros2_mara_ws/install/setup.bash
ros2 run individual_trajectories_bridge individual_trajectories_bridge -motors `ros2 pkg prefix individual_trajectories_bridge`/share/individual_trajectories_bridge/motors.yaml
```

###### Terminal 2:

This ROS 2.0 node will fetch all the `hros_actuation_servomotor_*********/state` topics define in the config file and these topics data will be republish in a topic called `/mara_controller/state`. This node also will be subscribe to `/mara_controller/command` and it will republish the data in to the corresponding H-ROS topic `hros_actuation_servomotor_*********/goal`. To run this ROS 2.0 just type:

```
source ~/ros2_mara_ws/install/setup.bash
ros2 run hros_cognition_mara_components hros_cognition_mara_components -motors `ros2 pkg prefix hros_cognition_mara_components`/share/hros_cognition_mara_components/link_order.yaml
```

##### ROS

Right now we need the nodes to run MoveIT!. We should execute the following nodes for setting up the `robot_description` parameter, a node that handles `follow_joint_trajectory` topic, the MoveIT nodes and finally RVIZ to move the robot.

###### Terminal 1:

This node will set the `robot_description` parameter. MoveIT make use of this parameter to calculate the forward and inverse kinematics.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
roslaunch mara_bringup mara_bringup.launch
```
###### Terminal 2:

This node will handle the `follow_joint_trajectory` topic. This node is subscribed to this topic and it will republish the data in the corresponding `hros_actuation_servomotor_*********/trajectory`.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
rosrun mara_bringup follow_joints_individual_trajectory.py
```

###### Terminal 3:

This terminal will launch MoveIT.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
roslaunch mara_moveit_config mara_moveit_planning_execution.launch
```

###### Terminal 4:

If you want to use RVIZ to move the robot that we need to type the following instructions.

```bash
source ~/ros_mara_ws/install_isolated/setup.bash
roslaunch mara_moveit_config moveit_rviz.launch config:=true
```

### Others

Convert URDF into sdf

```
xacro --inorder /home/erle/ros2_mara_ws/src/MARA/mara_description/urdf/mara_robot_camera_top.urdf.xacro -o /home/erle/ros2_mara_ws/src/MARA/mara_description/urdf/mara_robot_camera_top.urdf
gz sdf -p /home/erle/ros2_mara_ws/src/MARA/mara_description/urdf/mara_robot_camera_top.urdf > mara_robot_camera_top.sdf
```

### Example code

 - [mara_examples](https://github.com/AcutronicRobotics/mara_examples.git)

### Help

If you need help with MARA's real robot or its simulation, feel free to raise an issue [here](https://github.com/AcutronicRobotics/MARA/issues).
