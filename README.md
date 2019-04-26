# MARA

<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/products/mara/images/xv2_MARA2-11.jpg.pagespeed.ic.QRaRP5N01r.webp" align="left" hspace="8" vspace="2" width="270"></a>
<br/>
<br/>
<div style="text-align: justify">
<p>
This is the official repository of <a href="https://acutronicrobotics.com/products/mara/">MARA</a> modular robot, world's first modular cobot. MARA is the first robot which runs ROS 2.0 on each joint empowering new possibilities and applications in the professional and industrial landscapes of robotics. Built out of individual modules that natively run ROS 2.0, the modular robot arm can be physically extended in a seamless manner. MARA delivers industrial-grade features such as time synchronization or deterministic communication latencies.
</p>
<p>
Among other things, you will find in this repository instructions on how to simulate and control MARA in Gazebo Simulator or on the real robot.
</p>
</div>
<br/>
<a href="http://www.acutronicrobotics.com"><img src="https://acutronicrobotics.com/assets/images/AcutronicRobotics_logo_no_border.png" align="left" hspace="8" vspace="2" width="200"></a>
<br/>
<br/>
<br/>
<br/>
<br/>
<br/>

### Features

- **Powered by ROS 2.0**: a fully distributed software and hardware robotic architecture.

- **Highly customizable**: with daisy chaining, power and communication are exposed at the module level allowing for simplified extensions.

- **Real time data monitoring**: every H-ROS module is able to monitorize a variety of intrinsic aspects in real-time.

- **Power readings**: instantaneous voltage, current and power readings from each module, individually.

- **Automatic re-configuration**: embedded accelerometers, magnetometers and gyroscopes empower each module with inertial data.

- **HW and SW life cycle**: life cycle for each module allows greater control over the state of the ROS 2.0 system and the underlying components.

- **Controllable from any ROS 2.0 enabled computer**: [ORC](https://acutronicrobotics.com/products/orc/) is the ideal complement for MARA, but not mandatory. Choose yourself how you steer MARA.

<br/>

## Table of Contents

* [Specifications](#specifications)
* [Packages](#packages)
* [Install](#install)
  * [Dependendent tools](#dependent-tools)
  * [Create a ROS 2.0 workspace](#create-a-ros-20-workspace)
  * [Compile the ROS 2.0 workspace](#compile-the-ros-20-workspace)
  * [MoveIt! in ROS (Optional)](#moveit-in-ROS-optional)
* [Gazebo](#gazebo)
* [RViz](#rviz)
* [MoveIt!](#moveit)
  * [MoveIt! with MARA - Simulation](#moveit-with-mara---simulation)
    * [Terminal 1 (ROS 2.0):](#terminal-1-ros-20)
    * [Terminal 2 (ROS):](#terminal-2-ros)
    * [Terminal 3 (bridge):](#terminal-3-bridge)
  * [MoveIt! with MARA - Real](#moveit-with-mara---real-robot)
    * [Terminal 1 (ROS 2.0):](#terminal-1-ros-20)
    * [Terminal 2 (ROS):](#terminal-2-ros)
    * [Terminal 3 (bridge):](#terminal-3-bridge)
* [Examples](#examples)
* [Help](#help)

<br/>

## Specifications

![](https://acutronicrobotics.com/images/ACUTRONIC_modular_robot.png)

| Spec | Value |
|------|-------|
| Degrees of freedom | 6 DoF, extensible |
| Maximum speed | 90º/s |
| Repeatability | ±0.1 mm |
| Rated torque | 9.4/30/49 Nm |
| Payload | 3 Kg |
| Weight | 21 Kg |
| Height | 871 mm |
| Reach | 656 mm |
| Footprint | 204 mm |
| Robotics framework | ROS 2.0 Crystal Clemmys |
| Communication interfaces | 1 Gbps Ethernet, Compliant with TSN standards |
| Information model | Hardware Robot Information Model (HRIM®), version Coliza  |
| Security | Encrypted and secure computing environment, Secure data exchange capabilities &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; |
| Automatic updates | Over-the-Air (OTA) |
| Datasheet | [Download datasheet](https://acutronicrobotics.com/products/mara/files/MARA_datasheet_v1.1.pdf) |

<br/>

## Packages

In this section we will install all the necessary dependencies in order to be able to launch MARA.

- `hros_cognition_mara_components`: Transformations between JointTrajectory messages and module specific HRIM messages.
- `individual_trajectories_bridge`: Bridge to connect ROS and ROS 2.0.
- `mara_bringup`: roslaunch scripts for starting the MARA.
- `mara_contact_publisher`: ROS 2.0 publisher to know if a collision takes place.
- `mara_description`: 3D models of the MARA for simulation and visualization.
- `mara_gazebo`: Gazebo simulation package for the MARA.
- `mara_gazebo_plugins`: MARA Gazebo plugins for sensors and motors.
- `mara_utils_scripts`: Some scripts to move the MARA or spawn the model.

<br/>

## Install

The installation consists of two steps. The first one takes care of installing Gazebo which is necessary for trying out the MARA robot in a simulated environment. Since the MARA robot publishes all its relevant information via ROS 2.0 topics, installation of ROS 2.0 is also necessary in order to establish communication between user's PC and the robot.

### ROS 2.0 and Gazebo
- **Gazebo 9**: following the official instructions, [one-liner or step-by-step](http://gazebosim.org/tutorials?tut=install_ubuntu)
- **ROS 2.0 Crystal**: following the official instructions, [source](https://index.ros.org/doc/ros2/Linux-Development-Setup/) or [debian packages](https://index.ros.org/doc/ros2/Linux-Install-Debians/).

### Dependent tools

```sh
sudo apt install -y python3-vcstool python3-numpy wget ros-melodic-moveit-ros-move-group ros-melodic-moveit-visual-tools
```

### Create a ROS 2.0 workspace
Create the workspace and download source files:

```sh
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws
wget https://raw.githubusercontent.com/acutronicrobotics/MARA/master/mara-ros2.repos
vcs import src < mara-ros2.repos
```

Install and generate [HRIM](https://github.com/AcutronicRobotics/HRIM) dependencies:

```sh
cd ~/ros2_mara_ws/src/HRIM
sudo pip3 install hrim
hrim generate models/actuator/servo/servo.xml
hrim generate models/actuator/gripper/gripper.xml
```

### Compile the ROS 2.0 workspace

Please  make sure you are not sourcing ROS workspaces via `.bashrc` or any other way.

```sh
source /opt/ros/crystal/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-skip individual_trajectories_bridge
```
**Installation completed!** Now make sure you check the **[Examples](#examples)** section! Or follow the MoveIt! installation.

### MoveIt! in ROS (Optional)
MoveIt! is well known motion planning framework in the robotics community. MoveIt! allows to leverage different motion planners as well evaluate concepts such as manipulation, 3D perceptions, kinematics control and navigation in easy, user friendly way.
While MoveIt2! is not released (we are actively developing and contributing towards that effort), we provide the option to use ROS MoveIt! through bridges.

Continue the following steps to complete the MoveIt! installation.

#### ROS and MoveIt!
ROS and MoveIt! are required if you want to use `ìndividual_trajectories_bridge` to control the MARA, which means using ROS Melodic with MoveIt through bridges.
- **ROS melodic**: following the official instructions, [source](http://wiki.ros.org/melodic/Installation/Source) or [debian_packages](http://wiki.ros.org/melodic/Installation/Ubuntu).
- **MoveIt!**: `sudo apt install ros-melodic-moveit`.

#### ROS - ROS 2.0 Bridge
Compile the trajectory bridge located in the workspace using ROS as source.

```sh
source /opt/ros/melodic/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-select individual_trajectories_bridge
# Building ROS 1 creates conflicts with this ROS 2.0 workspace. Next line ensures the workspace is completely ROS 2.0.
sed -i 's#/opt/ros/melodic#/opt/ros/crystal#g' ~/ros2_mara_ws/install/setup.bash
```
#### ROS Workspace
Compile the MARA_ROS1 packages.
```sh
mkdir -p ~/ros_mara_ws/src
cd ~/catkin_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA_ROS1
cd ~/catkin_mara_ws/
catkin_make_isolated --install
```

<br/>

## Gazebo
You will find complimentary information in our [documentation's simulation section](https://acutronicrobotics.com/docs/technology/h-ros/api/level1/simulation).

Launch MARA!

```sh
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_gazebo mara.launch.py
```

**Optionally**, you can launch one of these launch files, which correspond to different grippers.

```sh
ros2 launch mara_gazebo mara_gripper_140.launch.py
ros2 launch mara_gazebo mara_gripper_85.launch.py
ros2 launch mara_gazebo mara_gripper_hande.launch.py
```

## RViz
### Launching MARA's Simulation

Skip this step if you are working with the real MARA. 

You can choose one of the available launch files. Let's launch MARA with the Robotiq's S140 Gripper for instance:

```bash
ros2 launch mara_gazebo mara_gripper_140.launch.py
```

### Visualizing MARA in RViz2

3D model visualization via robot_description topic will be supported in the upcoming ROS2 Dashing debian packages ([Rviz2 Issue](https://github.com/ros2/rviz/issues/395)). It is possible to [compile  RViz from sources](https://github.com/ros2/rviz#building-rviz-in-a-separate-workspace) or load the 3D model via URDF file.

```bash
rviz2 -d `ros2 pkg prefix mara_description`/share/mara_description/rviz/visualization.rviz
```

## MoveIt!
Motion planning, manipulation, 3D perception, kinematics, control and navigation through brigdes.

### MoveIt! with MARA - Simulation
Plan trajectories in a virtual environment with Gazebo and MoveIt!.

#### Terminal 1 (ROS 2.0)

Launch MARA:

```sh
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_gazebo mara.launch.py
```

**Optionally**, you can launch one of these launch files, which correspond to different grippers.

```sh
ros2 launch mara_gazebo mara_gripper_140.launch.py
ros2 launch mara_gazebo mara_gripper_85.launch.py
ros2 launch mara_gazebo mara_gripper_hande.launch.py
```

#### Terminal 2 (ROS)
```sh
source ~/catkin_mara_ws/devel/setup.bash

python3 ~/catkin_mara_ws/src/MARA_ROS1/mara_bringup/scripts/follow_joints_trajectory_actions.py ~/catkin_mara_ws/src/MARA_ROS1/mara_bringup/config/motors.yaml &
# change the prefix to match with the gripper used in the Terminal 1
roslaunch mara_bringup mara_bringup_moveit_actions.launch prefix:=140 &

```

#### Terminal 3 (bridge)
```sh
source ~/catkin_mara_ws/devel/setup.bash
source ~/ros2_mara_ws/install/setup.bash

ros2 run individual_trajectories_bridge individual_trajectories_bridge_actions -motors ~/ros2_mara_ws/src/mara/individual_trajectories_bridge/config/motors_actions.yaml
```

### MoveIt! with MARA - Real Robot
Plan trajectories in a real environment with MoveIt!.

#### Terminal 1 (ROS 2.0)
```sh
source ~/ros2_mara_ws/install/setup.bash
# you will need to change the export values according to the SoMs configuration
export RMW_IMPLEMENTATION=rmw_opensplice_cpp
export ROS_DOMAIN_ID=22

ros2 launch mara_bringup mara_bringup_real.launch.py
```

#### Terminal 2 (ROS)
```sh
source ~/catkin_mara_ws/devel/setup.bash
# you will need to change the yaml files to match the topics names on your SoMs
python3 ~/catkin_mara_ws/src/MARA_ROS1/mara_bringup/scripts/follow_joints_trajectory_actions.py ~/catkin_mara_ws/src/MARA_ROS1/mara_bringup/config/motors.yaml &
roslaunch mara_bringup mara_bringup_moveit_actions.launch prefix:=140 &

```

#### Terminal 3 (bridge)
```sh
source ~/catkin_mara_ws/devel/setup.bash
source ~/ros2_mara_ws/install/setup.bash
# you will need to change the export values according to the SoMs configuration, same as in Terminal 1
export RMW_IMPLEMENTATION=rmw_opensplice_cpp
export ROS_DOMAIN_ID=22
# you will need to change the yaml files to match the topics names on your SoMs
ros2 run individual_trajectories_bridge individual_trajectories_bridge_actions -motors ~/ros2_mara_ws/src/mara/individual_trajectories_bridge/config/motors_actions.yaml &
```
<br/>

## Examples

 - [Documentation and tutorials](https://acutronicrobotics.com/docs/products/robots/mara)
 - [mara_examples](https://github.com/AcutronicRobotics/mara_examples.git)

<br/>

## Help

If you need help with MARA's real robot or its simulation, feel free to raise an issue [here](https://github.com/AcutronicRobotics/MARA/issues).
