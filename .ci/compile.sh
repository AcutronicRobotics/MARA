#!/bin/bash

set -e

rm -rf src/MARA #Remove downloaded MARA to build current Branch
mv /tmp/MARA src/MARA
unset ROS_DISTRO
source /opt/ros/$ROS2_DISTRO/setup.bash
colcon build --merge-install --packages-skip individual_trajectories_bridge
# Build ROS-ROS 2.0 Bridge
#source /opt/ros/$ROS_DISTRO/setup.bash
#colcon build --merge-install --packages-select individual_trajectories_bridge
#sed -i 's#/opt/ros/melodic#/opt/ros/crystal#g' ~/ros2_mara_ws/install/setup.bash
#Build ROS workspace
#mkdir -p /root/catkin_mara_ws/src
#cd /root/catkin_mara_ws/src
git clone https://github.com/AcutronicRobotics/MARA_ROS1
#cd ..
#catkin_make_isolated --install
