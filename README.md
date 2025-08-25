# Home Service Robot Project 🏡

This repository contains the files for Project 5 of the Udacity Robotics Nanodegree, which focuses on creating a home service robot.

The project was particularly challenging due to the deprecation of key packages for older ROS distributions (Ubuntu 16 and ROS Kinetic). This version has been successfully implemented using **Ubuntu 20.04** and **ROS Noetic**, providing a more modern and stable environment.

---

## Environment and Setup

### Installation

Follow these steps to set up the project on your local machine.

1.  Clone the repository:
    ```bash
    git clone https://github.com/Fadi88/RSEND_path_planing.git
    ```
2.  Navigate to the project's root directory:
    ```bash
    cd RSEND_path_planing
    ```
3.  Run the setup script:
    ```bash
    ./setup.sh
    ```
This script handles all necessary dependencies, performs a `catkin_make` to build the workspace, and sources the environment, ensuring the project is ready to run.

---

## Nodes and Architecture

The project's architecture is built upon the standard ROS Navigation Stack and incorporates two custom C++ nodes to manage high-level tasks. All necessary nodes, including those for Turtlebot, SLAM, and Navigation, are installed automatically by the setup script.

### **`add_markers`**
This custom C++ node is responsible for defining and publishing navigation goals. 🎯
* **Functionality**: This node publishes a **`visualization_msgs/Marker`** message to a specific ROS topic. This marker serves as the navigation goal for the robot.
* **Behavior**: When the robot successfully reaches the target, this node detects the robot's proximity and removes the marker from the visualization, indicating that the task is complete.

### **`pick_objects`**
This custom C++ node acts as the main mission controller.
* **Functionality**: This node **subscribes** to the topic published by `add_markers`. When a new marker is received, it sends a goal to the **`move_base` action server**, instructing the robot to navigate to the marker's pose.
* **Behavior**: Once the robot successfully reaches the target, the node automatically commands the robot to return to a pre-defined **home position**. This creates an autonomous pick-and-place loop, simulating a service task.

### **`gmapping`**
This is a standard ROS node that provides **Simultaneous Localization and Mapping (SLAM)**. 
* **Functionality**: It uses a laser scanner and odometry data to build a 2D map of the environment in real time while simultaneously tracking the robot's pose within that map. The node publishes the map on the `/map` topic and the robot's pose on the TF tree, providing essential data for navigation.

### **`move_base`**
This is the core of the ROS Navigation Stack.
* **Functionality**: It takes a goal pose and, using a series of planners and costmaps, generates the velocity commands to drive the robot to the goal while avoiding obstacles. It uses a **global planner** for long-term pathfinding and a **local planner** for real-time obstacle avoidance.

---

## **Scripts**

All test scripts are located in the `catkin_ws` folder.

* The `test_slam.sh` and `test_navigation.sh` scripts have been verified and work as instructed.
* The `pick_object.sh` and `add_markers.sh` scripts will not work as their underlying nodes were modified and integrated into the final project's core functionality.

To run the final project, execute the `setup.sh` script from the project's root directory:

```bash
cd catkin_ws
./setup.sh