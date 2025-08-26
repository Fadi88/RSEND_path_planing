#!/bin/bash

# Source the ROS environment.
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
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

echo "Launching my robot simulation..."
roslaunch my_robot world.launch &
PIDS+=($!)
sleep 60

echo "Launching gmapping..."
roslaunch my_robot localization.launch &
PIDS+=($!)

rosrun pick_objects pick_objects_server &
rosrun add_markers add_markers_server &
sleep 5

echo "phase 1 : pick up"
rosservice call /update_marker -- 6.0 -2.5
sleep 0.1
rosservice call /set_and_navigate
rosservice call /delete_marker
sleep 5 # pickup time

echo "phase 2 : return"
rosservice call /update_marker -- 0.0 0.0
sleep 0.1
rosservice call /set_and_navigate
rosservice call /delete_marker

read -p ""