FROM osrf/ros:noetic-desktop-full
RUN apt update
RUN apt-get install -y \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    ros-noetic-effort-controllers python3-catkin-tools
ARG IP=0.0.0.0
ENV ROS_IP=$IP
RUN mkdir /root/src