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

echo "Launching TurtleBot3 simulation..."
roslaunch turtlebot3_gazebo turtlebot3_world.launch &
PIDS+=($!)
sleep 5  

echo "Launching Rviz with Gmapping visualization..."
gnome-terminal --window --title="Gmapping Rviz" -- bash -c "roslaunch turtlebot3_navigation turtlebot3_navigation.launch" &

rostopic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped \
"header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: "map"
pose:
  pose:
    position:
      x: -2.0
      y: -0.5
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.68539700]
"
sleep 2
rosrun add_markers add_markers &
rosrun pick_objects pick_objects &

read -p ""