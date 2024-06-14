#!/bin/bash

USERNAME="ubuntu"
PROJECT_NAME="team1"
WORKDIR_PATH=/home/${USERNAME}/${PROJECT_NAME}

# Update package lists and install essential packages
sudo apt-get update && \
sudo apt-get upgrade -y && \
sudo apt-get install -y \
    curl \
    wget \
    gnupg \
    lsb-release

# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS Noetic Desktop Full and additional dependencies
sudo apt-get update && \
sudo apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-effort-controllers \
    ros-noetic-teleop-twist-keyboard \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
sudo rosdep init && \
rosdep update

# ROS Build
# source /opt/ros/noetic/setup.bash && catkin_make
source /opt/ros/noetic/setup.bash && catkin build

# Set up environment variables and aliases
# echo -e "alias cm='cd ${WORKDIR_PATH} && catkin_make'" >> /home/${USERNAME}/.bashrc
echo "alias cb='cd ${WORKDIR_PATH} && catkin build'" >> /home/${USERNAME}/.bashrc
echo "alias cs='cd ${WORKDIR_PATH}/src'" >> /home/${USERNAME}/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc
echo "source ${WORKDIR_PATH}/devel/setup.bash" >> /home/${USERNAME}/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> /home/${USERNAME}/.bashrc
echo "export ROS_HOSTNAME=localhost" >> /home/${USERNAME}/.bashrc
echo "alias killgazebo='killall -9 gazebo & killall -9 gzserver & killall -9 gzclient'" >> /home/${USERNAME}/.bashrc
echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${WORKDIR_PATH}/devel/lib" >> /home/${USERNAME}/.bashrc

# Source .bashrc in the current shell
source /home/${USERNAME}/.bashrc
