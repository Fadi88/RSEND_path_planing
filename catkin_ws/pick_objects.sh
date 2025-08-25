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
  pkill -f "gzserver"
  pkill -f "gzclient"
  exit 0
}

trap cleanup EXIT

echo "Launching TurtleBot3 simulation..."
roslaunch my_robot world.launch &
PIDS+=($!)
sleep 5  

echo "Launching gmapping..."
roslaunch my_robot localization.launch &
PIDS+=($!)

rosrun pick_objects pick_objects &

read -p ""