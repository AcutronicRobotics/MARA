#!/bin/bash
: '
Author: Lander Usategui, lander at erlerobotics dot com
'

set -e

# Export colors
export RED="\033[31m"
export GREEN="\033[32m"
export YELLOW="\033[33m"
export BLUE="\033[34m"
export PURPLE="\e[0;35m"
export RESET="\e[0m"

# Path and branches
export ROS2_DISTRO="dashing"
export ROS1_DISTRO="melodic"
export MARA_ROS1_BRANCH="dashing"
export WS="/root/ros2_mara_ws"
export ROS1_WS="/root/catkin_ws"
export ROS2_SOURCE="/opt/ros/${ROS2_DISTRO}/"
export ROS1_SOURCE="/opt/ros/${ROS1_DISTRO}/"

# Colcon flags
export COLCON_COMMAND_FLAGS="--merge-install"
export COLCON_SELECT_PACKAGE="individual_trajectories_bridge"


function prepare_ws()
{
  echo -e "${YELLOW}Download dependencies for current branch${RESET}"
  cd ${WS} || exit
  rm -rf mara-ros2.repos
  cp -r /tmp/mara src && cp src/mara/mara-ros2.repos .
  sed -i '2,5d' mara-ros2.repos
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
  echo -e "${YELLOW}Making sure that everything is installed${RESET}"
  cd ${WS} || exit
  apt update -qq && rosdep update
  rosdep install -q -y --from-paths . --ignore-src --rosdistro \
                                  "${ROS2_DISTRO}" --skip-keys \
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
  cd ${WS}/src/HRIM || exit
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
  cd ${WS} || exit
  echo -e "${YELLOW}###### Packages to be compiled ######${RESET}"
  echo -e "${PURPLE}"
  colcon list --names-only
  echo -e "${RESET}"
  echo -e "${YELLOW}Compile the WS for ROS2${RESET}"
  source "${ROS2_SOURCE}setup.bash"
  colcon build "${COLCON_COMMAND_FLAGS}" --packages-skip "${COLCON_SELECT_PACKAGE}"
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
  source "${ROS1_SOURCE}setup.bash"
  cd ${WS} || exit
  colcon build "${COLCON_COMMAND_FLAGS}" --packages-select "${COLCON_SELECT_PACKAGE}"
  export ROS1_DISTRO=melodic && export ROS2_DISTRO=dashing
  sed -i "s#/opt/ros/${ROS1_DISTRO}#/opt/ros/${ROS2_DISTRO}#g" ${WS}/install/setup.bash
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
 cd "${ROS1_WS}"/src || exit
 # Check if branch exists
 git ls-remote --heads https://github.com/AcutronicRobotics/MARA_ROS1 | grep "${MARA_ROS1_BRANCH}" >/dev/null
 result=$?
 if [ $result -eq 1 ]; then
   echo -e "${RED}Cannot locate branch for MARA_ROS1${RESET}"
   exit $result
 else
   # Branch exits try to download It
   tries=0
   echo -e "${YELLOW}Downloading MARA_ROS1${RESET}"
   while [ "$(git clone -b "${MARA_ROS1_BRANCH}" https://github.com/AcutronicRobotics/MARA_ROS1)" ]; do
     echo -e "${YELLOW}Trying to download MARA_ROS1, try number: ${tries}${RESET}"
    /bin/sleep 2
    if [ $tries -eq 3 ]; then
      echo -e "${RED}Cannot download the MARA_ROS1${RESET}"
      break
    else tries=$((tries+1))
    fi
   done
 fi
 cd ${ROS1_WS} || exit
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
