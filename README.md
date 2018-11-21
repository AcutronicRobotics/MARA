# modular articulated arm - mara

## Install 


### Install ROS kinetic

Install ROS kinetic (desktop-full) following the official [instructions](http://wiki.ros.org/kinetic/Installation/Ubuntu).


### Create mara ROS workspace

Create a ROS workspace, for example:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Clone MARA repository in the workspace:

```
$ git clone https://github.com/erlerobot/mara/tree/master
```

### Add packages to the workspace source

In the src folder add the following extra packages:


Add general-message-pkgs:   
    
```
$ git clone  https://github.com/JenniferBuehler/general-message-pkgs
```

Add ros-industrial core pkgs:

```
$ git clone https://github.com/ros-industrial/industrial_core
```

Add Cartesian Path Planner MoveIt Plug-in pckg:

```
$ git clone -b indigo-devel https://github.com/ros-industrial-consortium/fermi/
```


Add descartes Cartesian Path Planner:

```
$ git clone https://github.com/ros-industrial-consortium/descartes
```


### Install dependencies:


Install controller-manager:

```
$ sudo apt install ros-kinetic-controller-manager
```

Install gazebo 9 following the official [instructions](http://gazebosim.org/tutorials?tut=ros_installing).

Install gazebo ros packgages:

```
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

$ sudo apt install ros-kinetic-gazebo9-ros ros-kinetic-gazebo9-plugins ros-kinetic-gazebo9-ros-control
```

Install `ros_control` controllers:

```
$ sudo apt-get install ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller
```

Install moveit!:

```
$ sudo apt-get install ros-kinetic-moveit
```

### Build workspace

Source in the workspace:

```
$ cd ~/catkin_ws/src
$ source /opt/ros/kinetic/setup.bash
$ catkin_init_workspace
$ cd ..
$ catkin_make
```

## MoveIt! with a simulated mara

#### Gazebo

```
$ roslaunch mara_gazebo mara.launch--> NO
$ roslaunch mara_gazebo mara_demo_camera_top.launch
$ roslaunch mara_gazebo mara_demo_camera_side.launch
```

#### MoveIT

```
$ roslaunch mara_moveit_config mara_moveit_planning_execution.launch sim:=true
```

#### RVIZ
```
$ roslaunch mara_moveit_config moveit_rviz.launch config:=true
```
