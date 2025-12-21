#!/bin/bash
set -e

TARGET_DIR=~/altair_project

echo "Targeting: $TARGET_DIR"

if [ -d "$TARGET_DIR/.git" ]; then
    echo "Repo exists, pulling..."
    cd $TARGET_DIR
    git pull
else
    echo "Fresh clone required."
    if [ -d "$TARGET_DIR" ]; then
        echo "Directory exists but is not a repo. Backing up..."
        mv $TARGET_DIR "${TARGET_DIR}_bak_$(date +%s)"
    fi
    echo "Cloning..."
    git clone https://github.com/Meta-Develop/ALTAIR_DRONE.git $TARGET_DIR
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install Dependencies
cd $TARGET_DIR/software
echo "Installing dependencies..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "Initializing rosdep..."
    sudo rosdep init || true
fi
export PATH=$PATH:/usr/bin
echo "PATH: $PATH"
which rosdep
rosdep update
rosdep install --from-paths . --ignore-src -r -y

# Build Node C Workspace
echo "Building RPi4_2_ws..."
cd RPi4_2_ws
colcon build

# Setup Environment
if ! grep -q "source $TARGET_DIR/software/RPi4_2_ws/install/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source $TARGET_DIR/software/RPi4_2_ws/install/setup.bash" >> ~/.bashrc
fi

echo "Setup Complete!"
