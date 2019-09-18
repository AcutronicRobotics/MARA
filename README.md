# MARA

[![Build
Status](https://travis-ci.org/AcutronicRobotics/MARA.svg?branch=dashing)](https://travis-ci.org/AcutronicRobotics/MARA)

This is the official repository of <a href="https://acutronicrobotics.com/products/mara/">MARA</a> modular robot, world's first modular cobot. MARA is the first robot which runs ROS 2.0 on each joint empowering new possibilities and applications in the professional and industrial landscapes of robotics. Built out of individual modules that natively run ROS 2.0, the modular robot arm can be physically extended in a seamless manner. MARA delivers industrial-grade features such as time synchronization or deterministic communication latencies.
</p>
<p>
Among other things, you will find in this repository instructions on how to simulate and control MARA in Gazebo Simulator or on the real robot.
</p>
</div>

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
  * [ROS 2.0](#ros-20)
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
    * [Terminal 1 (ROS 2.0):](#terminal-1-ros-20-1)
    * [Terminal 2 (ROS):](#terminal-2-ros-1)
    * [Terminal 3 (bridge):](#terminal-3-bridge-1)
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
| Robotics framework | ROS 2.0 Dashing Diademata  |
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

### ROS 2.0

- **ROS 2.0 Dashing**: following the official instructions, [source](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/) or [debian packages](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/). Make sure that you have colcon in your machine if you are installing from Debian packages.
`sudo apt install python3-colcon-common-extensions`

## Dependent tools
- **Gazebo 9.9.0**.
   - Install the latest available version of Gazebo via [one-liner instructions](http://gazebosim.org/tutorials?tut=install_ubuntu#Defaultinstallation:one-liner). Lower versions like **9.0.0 will not work**. Additional information is available [here](https://github.com/AcutronicRobotics/gym-gazebo2/issues/31#issuecomment-501660211).
     ```sh
     curl -sSL http://get.gazebosim.org | sh
     ```
- ROS 2 extra packages
```sh
sudo apt update && sudo apt install -y \
ros-dashing-rttest \
ros-dashing-rclcpp-action \
ros-dashing-gazebo-dev \
ros-dashing-gazebo-msgs \
ros-dashing-gazebo-plugins \
ros-dashing-gazebo-ros \
ros-dashing-gazebo-ros-pkgs

sudo apt install -y \
python3-pip python3-vcstool python3-numpy wget python3-pyqt5 python3-colcon-common-extensions git

```

### Create a ROS 2.0 workspace
Create the workspace and download source files:

```sh
mkdir -p ~/ros2_mara_ws/src
cd ~/ros2_mara_ws
wget https://raw.githubusercontent.com/acutronicrobotics/MARA/dashing/mara-ros2.repos
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
source /opt/ros/dashing/setup.bash
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

    Dependent tools:
    ```sh
    # ROS extra packages
    sudo apt update && sudo apt install -y \
    ros-melodic-xacro \
    ros-melodic-rviz \
    ros-melodic-control-msgs \
    ros-melodic-robot-state-publisher
    ```

- **MoveIt!**: Install the following ROS debian packages.
    ```sh
    sudo apt install -y \
    ros-melodic-moveit \
    ros-melodic-moveit-ros-move-group \
    ros-melodic-moveit-visual-tools
    ros-melodic-moveit-simple-controller-manager
    ```
 #### ROS - ROS 2.0 Bridge
Compile the trajectory bridge located in the workspace using ROS as source.

```sh
source /opt/ros/melodic/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-select individual_trajectories_bridge
# Building ROS 1 creates conflicts with this ROS 2.0 workspace. Next line ensures the workspace is completely ROS 2.0.
sed -i 's#/opt/ros/melodic#/opt/ros/dashing#g' ~/ros2_mara_ws/install/setup.bash
```
#### ROS Workspace
Compile the MARA_ROS1 packages.
```sh
mkdir -p ~/catkin_mara_ws/src
cd ~/catkin_mara_ws/src
git clone -b dashing https://github.com/AcutronicRobotics/MARA_ROS1
cd ~/catkin_mara_ws/
catkin_make_isolated --install
```

<br/>

## Gazebo
Complementary information is available in our [documentation's simulation section](https://acutronicrobotics.com/docs/technology/h-ros/api/level1/simulation).

```sh
source ~/ros2_mara_ws/install/setup.bash
ros2 launch mara_gazebo mara.launch.py
```

**Optionally**, you can launch a different versions of MARA robot using the `--urdf` flag to indicate the desired urdf to be spawned:

```sh
ros2 launch mara_gazebo mara.launch.py --urdf mara_robot_gripper_140
```

*Available urdfs: `mara_robot_gripper_140`, `mara_robot_gripper_140_no_table`, `mara_robot_gripper_85`, `mara_robot_gripper_hande`, `two_mara_robots` and `two_mara_robots_gripper_140_no_table`*

<br/>

## RViz

First, you will need to [launch MARA in Gazebo](#gazebo) in another terminal.

3D model visualization via robot_description topic will be supported in the upcoming ROS2 Dashing debian packages ([Rviz2 Issue](https://github.com/ros2/rviz/issues/395)). We recommend to [compile  RViz from sources](https://github.com/ros2/rviz#building-rviz-in-a-separate-workspace) in the meantime.

```bash
source ~/rviz2_ws/install/setup.bash
source ~/ros2_mara_ws/install/setup.bash
rviz2 -d `ros2 pkg prefix mara_description`/share/mara_description/rviz/visualization.rviz
```

Alternatively, instead of using the `robot_description` topic, you can load the 3D model manually selecting the URDF file in the RobotModel section of RViz.

<br/>

## MoveIt!
Motion planning, manipulation, 3D perception, kinematics, control and navigation through brigdes.

### MoveIt! with MARA - Simulation
Plan trajectories in a virtual environment with Gazebo and MoveIt!.

#### Terminal 1 (ROS 2.0)
[Launch MARA in Gazebo](#gazebo).

#### Terminal 2 (ROS)
```sh
source ~/catkin_mara_ws/devel_isolated/setup.bash
roslaunch mara_bringup mara_bringup_moveit_actions.launch
```

If you have used a different urdf in the Terminal 1, you will need to use `urdf:=` to launch the same one:

```sh
roslaunch mara_bringup mara_bringup_moveit_actions.launch urdf:=mara_robot_gripper_140
```

*In case you have launched two robots, you will need to add `multiple_robots:=true`*

#### Terminal 3 (bridge)
Source *catkin_mara_ws* and *ros2_mara_ws*:
```sh
source ~/catkin_mara_ws/devel_isolated/setup.bash
source ~/ros2_mara_ws/install/setup.bash
```
Run the bridge:
```sh
ros2 run individual_trajectories_bridge individual_trajectories_bridge_actions -motors ~/ros2_mara_ws/src/mara/hros_cognition_mara_components/config/motors.yaml sim
```

If you have launched two mara robots, you will have to run the bridge in the following way:
```sh
ros2 run individual_trajectories_bridge individual_trajectories_bridge_actions -motors ~/ros2_mara_ws/src/mara/hros_cognition_mara_components/config/two_motors.yaml sim
```

### MoveIt! with MARA - Real Robot
Plan trajectories in a real environment with MoveIt!.

:warning: You will need to change the names of the real motors in [MARA/hros_cognition_mara_components](https://github.com/AcutronicRobotics/MARA/blob/dashing/hros_cognition_mara_components/config/motors.yaml#L16-L21) and in [MARA_ROS1/mara_bringup](https://github.com/AcutronicRobotics/MARA_ROS1/blob/dashing/mara_bringup/config/motors.yaml#L10-L15) files to match the MACs of your SoMs.

:warning: Any change in the yaml files you will have to recompile the ros2 and ros packages (make sure you source only the corresponding ros/ros2):
```sh
source /opt/ros/dashing/setup.bash
cd ~/ros2_mara_ws && colcon build --merge-install --packages-select hros_cognition_mara_components
```
```sh
source /opt/ros/melodic/setup.bash
cd ~/catkin_mara_ws && catkin_make_isolated --install --pkg mara_bringup
```

#### Terminal 1 (ROS 2.0)

```sh
source ~/ros2_mara_ws/install/setup.bash
# you will need to change the export values according to the SoMs configuration when running on the real robot.
export RMW_IMPLEMENTATION=rmw_opensplice_cpp
export ROS_DOMAIN_ID=22

ros2 launch mara_bringup mara.launch.py
```

If your real robot has any extra component or you want to control more than one robot, you will need to set the `--urdf` flag to indicate the urdf that corresponds to your real robot (environment):

```sh
ros2 launch mara_bringup mara.launch.py --urdf mara_robot_gripper_140
```

*Available urdfs: `mara_robot_gripper_140`, `mara_robot_gripper_140_no_table`, `mara_robot_gripper_85`, `mara_robot_gripper_hande`, `two_mara_robots` and `two_mara_robots_gripper_140_no_table`*

#### Terminal 2 (ROS)

```sh
source ~/catkin_mara_ws/devel_isolated/setup.bash
roslaunch mara_bringup mara_bringup_moveit_actions.launch env:=real
```

If you have used a different urdf in the Terminal 1, you will need to use `urdf:=` to launch the same one:

```sh
roslaunch mara_bringup mara_bringup_moveit_actions.launch env:=real urdf:=mara_robot_gripper_140
```

*If case you want to control two robots you will need to add `multiple_robots:=true`*


#### Terminal 3 (bridge)

Source *catkin_mara_ws* and *ros2_mara_ws*, and export `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID`:
```sh
source ~/catkin_mara_ws/devel_isolated/setup.bash
source ~/ros2_mara_ws/install/setup.bash
# you will need to change the export values according to the SoMs configuration, same as in Terminal 1
export RMW_IMPLEMENTATION=rmw_opensplice_cpp
export ROS_DOMAIN_ID=22
```
Run the bridge:
```sh
ros2 run individual_trajectories_bridge individual_trajectories_bridge_actions -motors ~/ros2_mara_ws/src/mara/hros_cognition_mara_components/config/motors.yaml real
```
If you have two mara robots, you will have to run the bridge in the following way:
```sh
ros2 run individual_trajectories_bridge individual_trajectories_bridge_actions -motors ~/ros2_mara_ws/src/mara/hros_cognition_mara_components/config/two_motors.yaml real
```

<br/>

## Examples

 - [Documentation and tutorials](https://acutronicrobotics.com/docs/products/robots/mara)
 - [mara_examples](https://github.com/AcutronicRobotics/mara_examples.git)

<br/>

## Help

If you need help with MARA's real robot or its simulation, feel free to raise an issue [here](https://github.com/AcutronicRobotics/MARA/issues).
