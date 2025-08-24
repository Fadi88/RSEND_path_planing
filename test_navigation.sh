#!/bin/bash

# Source the ROS environment.
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger

PIDS=()

# Function to kill all processes.
function cleanup {
  echo "Trapped exit signal. Killing all background ROS processes..."
  for pid in "${PIDS[@]}"; do
    if ps -p "$pid" > /dev/null; then
      kill "$pid" 2>/dev/null
    fi
  done
  pkill -f "roscore"
  exit 0
}

trap cleanup EXIT

echo "Launching TurtleBot3 simulation..."
roslaunch turtlebot3_gazebo turtlebot3_house.launch &
PIDS+=($!)
sleep 10

echo "Launching amcl demo..."
roslaunch roslaunch turtlebot3_navigation amcl.launch &
PIDS+=($!)

echo "Launching Rviz with Gmapping visualization..."
gnome-terminal --window --title="Gmapping Rviz" -- bash -c "roslaunch turtlebot3_navigation turtlebot3_navigation.launch" &

echo "Launching keyboard teleop in a new window. Use that window to drive the robot."
gnome-terminal --window --title="TurtleBot3 Teleop" -- bash -c "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" &

read -p ""