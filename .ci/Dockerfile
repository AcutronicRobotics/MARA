FROM ubuntu:bionic

LABEL maintainer="Lander Usategui <lander at erlerobotics dot com>"

ENV ROS2_DISTRO crystal
ENV ROS_DISTRO melodic
#Prepare work-space
RUN mkdir -p /root/ros2_mara_ws/src
WORKDIR /root/ros2_mara_ws

RUN \
    echo 'Etc/UTC' > /etc/timezone \
      && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
      && apt-get update -qq && apt-get install -qq -y tzdata dirmngr gnupg2 lsb-release curl \
      # setup ros2 keys
      && curl http://repo.ros2.org/repos.key | apt-key add - \
      # setup sources.list
      && echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
      && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
      && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 \
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
        ros-$ROS2_DISTRO-action-msgs \
        ros-$ROS2_DISTRO-message-filters \
        ros-$ROS2_DISTRO-yaml-cpp-vendor \
        ros-$ROS2_DISTRO-urdf \
        ros-$ROS2_DISTRO-rttest \
        ros-$ROS2_DISTRO-tf2 \
        ros-$ROS2_DISTRO-tf2-geometry-msgs \
        ros-$ROS2_DISTRO-rclcpp-action \
        ros-$ROS2_DISTRO-cv-bridge \
        ros-$ROS2_DISTRO-control-msgs \
        ros-$ROS2_DISTRO-image-transport \
        ros-$ROS2_DISTRO-gazebo-dev \
        ros-$ROS2_DISTRO-gazebo-msgs \
        ros-$ROS2_DISTRO-gazebo-plugins \
        ros-$ROS2_DISTRO-gazebo-ros \
        ros-$ROS2_DISTRO-gazebo-ros-pkgs \
        ros-$ROS_DISTRO-desktop \
        ros-$ROS_DISTRO-moveit \
        ros-$ROS_DISTRO-moveit-ros-move-group \
        ros-$ROS_DISTRO-moveit-visual-tools \
       && wget https://raw.githubusercontent.com/acutronicrobotics/MARA/master/mara-ros2.repos \
       && vcs import src < mara-ros2.repos \
       && pip3 install hrim \
       && cd src/HRIM && hrim generate models/actuator/servo/servo.xml && hrim generate models/actuator/gripper/gripper.xml
COPY ./compile.sh /root/ros2_mara_ws
ENTRYPOINT ["/root/ros2_mara_ws/compile.sh"]
