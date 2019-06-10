FROM ubuntu:bionic

LABEL maintainer="Lander Usategui <lander at erlerobotics dot com>"

ENV ROS2_DISTRO dashing
ENV ROS_DISTRO melodic
#Prepare work-space
RUN mkdir -p /root/ros2_mara_ws/src
WORKDIR /root/ros2_mara_ws

RUN \
    echo 'Etc/UTC' > /etc/timezone \
      && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
      && apt-get update -qq && apt-get install -qq -y tzdata dirmngr gnupg2 lsb-release curl \
      # setup ros2 keys
      && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
      # setup sources.list
      && echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
      && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
      && counter=1 \
      && while [ ! "$(apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116)" ]; do /bin/sleep 2; if [ $counter -eq 3 ]; then break;else counter=$((counter+1)); fi done \
      && apt-get update -qq && apt-get install -qq -y \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-pip \
        python3-vcstool \
        python3-numpy \
        python3-vcstool \
        wget \
        ros-$ROS2_DISTRO-ros-base \
        ros-$ROS_DISTRO-desktop \
        ros-$ROS_DISTRO-moveit \
        ros-$ROS_DISTRO-moveit-ros-move-group \
        ros-$ROS_DISTRO-moveit-visual-tools \
       && wget https://raw.githubusercontent.com/acutronicrobotics/MARA/master/mara-ros2.repos \
       && vcs import src < mara-ros2.repos \
       && pip3 install hrim \
       && cd src/HRIM && hrim generate models/actuator/servo/servo.xml && hrim generate models/actuator/gripper/gripper.xml && hrim generate models/sensor/forcetorque/forcetorque.xml
COPY ./compile.sh /root/ros2_mara_ws
#ENTRYPOINT ["/root/ros2_mara_ws/compile.sh"]
CMD ["bash"]
