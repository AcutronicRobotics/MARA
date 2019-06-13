#!/bin/bash

set -e

rm -rf src/mara/* #Remove downloaded MARA to build current Branch
cp -r /tmp/MARA/* src/mara
source /opt/ros/$ROS2_DISTRO/setup.bash
colcon build --merge-install --packages-skip individual_trajectories_bridge
# Build ROS-ROS 2.0 Bridge
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --merge-install --packages-select individual_trajectories_bridge
sed -i 's#/opt/ros/melodic#/opt/ros/dashing#g' /root/ros2_mara_ws/install/setup.bash
unset ROS_DISTRO
export ROS_DISTRO=melodic
source /opt/ros/$ROS_DISTRO/setup.bash
# Compile the MARA_ROS1 packages.
echo "Build MARA_ROS1"
mkdir -p ~/catkin_mara_ws/src
cd ~/catkin_mara_ws/src
git clone -b dashing https://github.com/AcutronicRobotics/MARA_ROS1
cd ~/catkin_mara_ws/
catkin_make_isolated --install
