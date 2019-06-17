FROM ros:dashing

LABEL maintainer="Lander Usategui <lander at erlerobotics dot com>"
ENV ROS1_DISTRO melodic
ARG BRANCH=""
#Prepare work-space
RUN mkdir -p /root/ros2_mara_ws/src
WORKDIR /root/ros2_mara_ws
RUN \
        echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list \
        && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
        && apt update && apt install -y \
          wget \
          ros-$ROS1_DISTRO-desktop \
          ros-$ROS1_DISTRO-moveit \
          ros-$ROS1_DISTRO-moveit-ros-move-group \
          ros-$ROS1_DISTRO-moveit-visual-tools \
       && wget https://raw.githubusercontent.com/acutronicrobotics/MARA/${BRANCH}/mara-ros2.repos \
       && vcs import src < mara-ros2.repos \
       && rosdep update -q \
       && rosdep install -q -y --from-paths . --ignore-src --rosdistro \
                              ${ROS_DISTRO} --skip-keys \
                             "hrim_actuator_rotaryservo_actions \
                              hrim_actuator_gripper_srvs \
                              hrim_actuator_rotaryservo_msgs hrim_generic_srvs \ 
                              hrim_generic_msgs hrim_actuator_gripper_msgs \
                              hrim_actuator_rotaryservo_srvs" \
                              --as-root=apt:false || true \
      && rm -rf /var/lib/apt/lists/*
RUN rm -rf /root/ros2_mara_ws/src/*
ENV TERM xterm
CMD ["bash"]
