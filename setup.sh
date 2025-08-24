#!/bin/bash

# This script automates the setup of ROS 1 Noetic and the TurtleBot3 simulation packages.

# Function to check for errors and exit
check_error() {
    if [ $? -ne 0 ]; then
        echo "Error: $1"
        exit 1
    fi
}

echo "Starting ROS Noetic and TurtleBot3 setup..."

# Install ROS 1 Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
check_error "Failed to set up sources.list."

sudo apt install curl -y
check_error "Failed to install curl."

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
check_error "Failed to add ROS key."

sudo apt update
check_error "Failed to update package list."

sudo apt install ros-noetic-desktop-full -y
check_error "Failed to install ROS Noetic."

# Handle rosdep init error if sources file already exists
if [ -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    echo "Rosdep sources file already exists, deleting it to prevent error."
    sudo rm "/etc/ros/rosdep/sources.list.d/20-default.list"
fi

sudo rosdep init
check_error "Failed to initialize rosdep."

rosdep update
check_error "Failed to update rosdep."

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
check_error "Failed to add ROS setup to .bashrc."

# Install TurtleBot3 and Gazebo Packages
sudo apt update
check_error "Failed to update package list."

sudo apt install xterm ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-slam-gmapping ros-noetic-gmapping -y
check_error "Failed to install Gazebo and Gmapping packages."

sudo apt install ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-dwa-local-planner -y
check_error "Failed to install TurtleBot3 packages."

# Set the TurtleBot3 model
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
check_error "Failed to set TurtleBot3 model in .bashrc."

# Source the .bashrc file to apply changes
source ~/.bashrc

echo "Setup complete! Please restart your terminal or run 'source ~/.bashrc' to ensure all changes are applied."
