# Home Service Robot Project 🏡

This repository contains the files for Project 5 of the Udacity Robotics Nanodegree, focused on creating a home service robot.

The project was particularly challenging due to the deprecation of key packages for Ubuntu 16 and ROS Kinetic. This version has been successfully implemented using **Ubuntu 20.04** and **ROS Noetic** instead.

## Environment and Setup

### Installation

Follow these steps to get a development environment up and running on your local machine.

1.  Clone the repository:

    ```bash
    git clone https://github.com/Fadi88/RSEND_path_planing.git
    ```

2.  Run the setup script:

    ```bash
    ./setup.sh
    ```

### Nodes

The project's setup scripts automatically install all necessary nodes, including those for Turtlebot, SLAM, and Navigation. The two custom nodes in this project are:

-   **`add_markers`**: Responsible for sending navigation goals to the robot.
-   **`pick_objects`**: Reads the markers and directs the robot to navigate to the specified location.

### Scripts

All test scripts are located in the `catkin_ws` folder.

-   The `test_slam.sh` and `test_navigation.sh` scripts have been verified to work as instructed.
-   The `pick_object-sh` and `add_markers.sh` scripts will not work as the underlying nodes were modified for the final project.

To run the final project, execute the following commands from the project's root directory:

```bash
cd catkin_ws
./setup.sh
```

This script will handle all necessary sourcing and `catkin_make` commands.
