# MARA

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/assets/images/AcutronicRobotics_logo.jpg" align="left" hspace="8" vspace="2" width="200"></a>

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
              * [Rviz2](#rviz2)
              * [MoveIT!](#moveit)
                  * [ROS 2.0](#ros-20)
                     * [Terminal 1:](#terminal-1-1)
                     * [Terminal 2:](#terminal-2-1)
                  * [ROS](#ros)
                     * [Terminal 1:](#terminal-1-2)
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

 - [gazebo_ros_pkgs](https://github.com/acutronicrobotics/gazebo_ros_pkgs) branch: `ros2_gym-gazebo`.
 - [HRIM](https://github.com/erlerobot/HRIM/).
 - [control_msgs](https://github.com/erlerobot/control_msgs) branch: `crystal`.
 - [image_common](https://github.com/ros-perception/image_common) branch: `crystal`.
 - [vision_opencv](https://github.com/ros-perception/vision_opencv) branch: `ros2`
 - sudo apt install python3-numpy


### Install

#### Install ROS 2.0

Install ROS 2.0 following the official instructions: [source](https://index.ros.org/doc/ros2/Linux-Development-Setup/) [debian packages](https://index.ros.org/doc/ros2/Linux-Install-Debians/).

#### Create mara ROS 2.0 workspace
Create a ROS workspace, for example:

```bash
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws
sudo apt install -y python3-vcstool python3-numpy
wget https://raw.githubusercontent.com/acutronicrobotics/MARA/master/mara-ros2.repos
vcs import src < mara-ros2.repos
```

Generate HRIM dependencies:

```bash
cd ~/ros2_mara_ws/src/HRIM/installator
python3 setup.py install && cd ..
hrim generate models/actuator/servo/servo.xml
hrim generate models/actuator/gripper/gripper.xml
```

#### Compile

Right now you can compile the code:

```bash
source /opt/ros/crystal/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-skip individual_trajectories_bridge
```

**Optional note**: If you want to use MoveIT! you need to source ROS 1.0 environment variables. Typically, if you have installed ROS `Melodic`, you need to source the following file and compile `individual_trajectories_bridge`:

```bash
source /opt/ros/melodic/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-select individual_trajectories_bridge
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

To bring up the simulated robot in Gazebo. You can choose one of the following ros2 launch depends on the gripper that you want to use:

```
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_gazebo mara.launch.py
ros2 launch mara_gazebo mara_gripper_140.launch.py
ros2 launch mara_gazebo mara_gripper_85.launch.py
ros2 launch mara_gazebo mara_gripper_hande.launch.py
```

##### Rviz2

To visualize the robot with Rviz2, you should type the following instructions:

```
source ~/ros2_mara_ws/install/setup.bash
rviz2
```

##### MoveIT!

###### ROS 2.0

###### Terminal 1:

You can run `gazebo`, spawn the model, publish the robot state and run `hros_cognition_mara_components` using the ROS 2.0 launch file. You can choose one of the following `ros2 launch` depends on the gripper that you want to use:

```
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_gazebo mara.launch.py
ros2 launch mara_gazebo mara_gripper_140.launch.py
ros2 launch mara_gazebo mara_gripper_85.launch.py
ros2 launch mara_gazebo mara_gripper_hande.launch.py
```

###### Terminal 2:

We need to run this node to create a bridge bewteen ROS and ROS 2.0. The topics that will be available are `/mara_controller/state`, `/joints_state` and `hros_actuation_servomotor_*********/trajectory`. Type the following command to run the bridge:

```
source /opt/ros/melodic/setup.bash
source ~/ros2_mara_ws/install/setup.bash
ros2 run individual_trajectories_bridge individual_trajectories_bridge -motors `ros2 pkg prefix individual_trajectories_bridge`/share/individual_trajectories_bridge/motors.yaml
```

###### ROS

You can run the four needed nodes using this launch file. You should use the argument `prefix` to indicate which gripper you are using `85`, `140` or `hande`:

```
source ~/ros_mara_ws/install_isolated/setup.bash
roslaunch mara_bringup mara_bringup_moveit.launch prefix:=85
```

### Example code

 - [mara_examples](https://github.com/AcutronicRobotics/mara_examples.git)

### Help

If you need help with MARA's real robot or its simulation, feel free to raise an issue [here](https://github.com/AcutronicRobotics/MARA/issues).
