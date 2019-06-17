#!/bin/bash
: '
Author: Lander Usategui, lander at erlerobotics dot com
'

# Export colors
export RED="\033[31m"
export GREEN="\033[32m"
export YELLOW="\033[33m"
export BLUE="\033[34m"
export RESET="\e[0m"

# Path and branches
export WS="/root/ros2_mara_ws"
export ROS1_WS="/root/catkin_ws"
export ROS2_SOURCE="/opt/ros/${ROS2_DISTRO}/"
export ROS1_SOURCE="/opt/ros/${ROS1_DISTRO}/"
export BRANCH="dashing"
export ROS2_DISTRO="dashing"
export ROS1_DISTRO="melodic"

# Colcon flags
export COLCON_COMMAND="--merge-install"
export COLCON_SKIP_PACKAGES="--packages-skip individual_trajectories_bridge"


function prepare_ws()
{
  echo -e "${YELLOW}Download dependencies for ${BRANCH}${RESET}"
  cd ${WS}
  rm -rf mara-ros2.repos
  wget https://raw.githubusercontent.com/AcutronicRobotics/MARA/${BRANCH}/mara-ros2.repos
  vcs import src < mara-ros2.repos 
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}prepare_ws failed${RESET}"
    exit $result
  else
    echo -e "${BLUE}prepare_ws function successfully finished${RESET}"
  fi
}

function run_rosdep()
{
  echo -e "${YELLOW}Make sure everything is installed${RESET}"
  cd ${WS}
  apt update -qq && rosdep update
  rosdep install -q -y --from-paths . --ignore-src --rosdistro \
                                  ${ROS_DISTRO} --skip-keys \
                                  "hrim_actuator_rotaryservo_actions \
                                   hrim_actuator_gripper_srvs \
                                   hrim_actuator_rotaryservo_msgs \
                                   hrim_generic_srvs \ 
                                   hrim_generic_msgs \
                                   hrim_actuator_gripper_msgs \
                                   hrim_actuator_rotaryservo_srvs" \
                                   --as-root=apt:false || true
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}run_rosdep failled${RESET}"
    exit $result
  else
    echo -e "${BLUE}All dependencies successfully installed${RESET}"
  fi
}

function setup_hrim()
{
  echo -e "${YELLOW}Installing hrim cli${RESET}"
  pip3 install hrim 
  cd ${WS}/src/HRIM
  hrim generate models/actuator/servo/servo.xml
  result=$?
  hrim generate models/actuator/gripper/gripper.xml
  result1=$?
  if [ $result -ne 0 ] || [ $result1 -ne 0 ]; then
    echo -e "${RED}HRIM installation failed${RESET}"
    exit $result
  else
    echo -e "${BLUE}HRIM successfully installed${RESET}"
  fi
}

function compile_ws()
{
  echo -e "${YELLOW}Compile the WS for ROS2${RESET}"
  cd ${WS}
  colcon build ${COLCON_COMMAND} ${COLCON_SKIP_PACKAGES}
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}Error compiling the ws${RESET}"
    exit $result
  else
    echo -e "${BLUE}WS compiled successfully${RESET}"
  fi
}

function compile_ros_bridge()
{
  echo -e "${YELLOW}Compile bridge for ROS melodic${RESET}"
  unset ROS_DISTRO && unset ROS2_DISTRO
  source ${ROS1_SOURCE}setup.bash
  cd ${WS}
  colcon build --merge-install --packages-select individual_trajectories_bridge
  export ROS1_DISTRO=melodic && export ROS2_DISTRO=dashing
  sed -i 's#/opt/ros/${ROS1_DISTRO}#/opt/ros/${ROS2_DISTRO}#g' ${WS}/install/setup.bash
  result=$?
  if [ $result -ne 0 ]; then
    echo -e "${RED}Error compiling the bridge${RESET}"
    exit $result
  else
    echo -e "${BLUE}Bridge successfully compiled${RESET}"
  fi
}

function ros1_ws()
{
 mkdir -p ${ROS1_WS}/src
 cd ${ROS1_WS}/src
 git clone -b ${BRANCH} https://github.com/AcutronicRobotics/MARA_ROS1 2>/dev/null
 result=$?
 if [ $result -ne 0 ]; then
   echo -e "${RED}Cannot locate branch for MARA_ROS1${RESET}"
   exit $result
 fi
 cd ${ROS1_WS}
 echo -e "${BLUE}Compile MARA_ROS1${RESET}"
 catkin_make_isolated --install
 result=$?
 if [ $result -ne 0 ]; then
   echo -e "${RED}ROS1_WS failed to compile${RESET}"
   exit $result
 else
   echo -e "${BLUE}ROS1_WS successfully compiled${RESET}"
 fi
}

prepare_ws
run_rosdep
setup_hrim
compile_ws
compile_ros_bridge
ros1_ws
#If we're here everything is okay
echo -e "${GREEN}All steps successfully completed${RESET}"
