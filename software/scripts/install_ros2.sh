#!/bin/bash
set -e

# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Detect OS and set ROS Distro
. /etc/os-release
if [ "$UBUNTU_CODENAME" = "noble" ]; then
    ROS_DISTRO="humble"
elif [ "$UBUNTU_CODENAME" = "jammy" ]; then
    ROS_DISTRO="humble"
else
    echo "Unsupported Ubuntu version: $UBUNTU_CODENAME"
    exit 1
fi

echo "Detected Ubuntu $UBUNTU_CODENAME. Installing ROS 2 $ROS_DISTRO."

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-ros-base
sudo apt install -y ros-dev-tools

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "ROS 2 $ROS_DISTRO installation complete."
