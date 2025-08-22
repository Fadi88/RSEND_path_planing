#!/bin/bash

# This script launches a ROS environment with a TurtleBot3, Gazebo, GMapping, RViz, and keyboard teleop.

# Set up DISPLAY for GUI apps
if [ -z "$DISPLAY" ]; then
    export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
fi
export LIBGL_ALWAYS_INDIRECT=1

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Check for required ROS packages before launching
echo "--- Checking for ROS packages ---"
echo "This may take a moment..."
if ! dpkg -s ros-noetic-turtlebot3-simulations >/dev/null 2>&1; then
    echo "ERROR: 'ros-noetic-turtlebot3-simulations' package not found. Please install it."
    exit 1
fi
if ! dpkg -s ros-noetic-gmapping >/dev/null 2>&1; then
    echo "ERROR: 'ros-noetic-gmapping' package not found. Please install it."
    exit 1
fi
echo "All required packages are installed. Continuing."

# Use xterm if available, otherwise fallback to a generic terminal emulator
if command -v xterm >/dev/null 2>&1; then
    TERMINAL_CMD="xterm"
else
    TERMINAL_CMD="x-terminal-emulator"
    echo "Note: xterm not found. Using generic terminal emulator instead."
fi

# Launch ROS Core
echo "--- Launching ROS Core ---"
$TERMINAL_CMD -hold -title "ROS Core" -e "bash -c 'source /opt/ros/noetic/setup.bash && roscore'" &
sleep 5

# Launch Gazebo world
echo "--- Launching Gazebo ---"
$TERMINAL_CMD -hold -title "Gazebo" -e "bash -c 'source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_gazebo turtlebot3_world.launch'" &
sleep 10

# Launch GMapping SLAM
echo "--- Launching GMapping ---"
$TERMINAL_CMD -hold -title "GMapping" -e "bash -c 'source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_slam turtlebot3_slam.launch'" &
sleep 5

# Launch RViz visualization
echo "--- Launching RViz ---"
$TERMINAL_CMD -hold -title "RViz" -e "bash -c 'source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_slam turtlebot3_slam.launch'" &
sleep 5

# Launch keyboard teleop
echo "--- Launching Teleop ---"
$TERMINAL_CMD -hold -title "Teleop" -e "bash -c 'source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'" &

echo "All components launched. Use the 'Teleop' window to control the robot."
