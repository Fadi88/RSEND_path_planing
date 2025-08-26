cmake_minimum_required(VERSION 3.0.2)
project(pick_objects)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  message_generation
  actionlib
  move_base_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetAndNavigate.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  visualization_msgs
)

## Declare a C++ executable
add_executable(pick_objects_server src/pick_objects.cpp)

## Add cmake target dependencies of the executable
## This ensures the service header file is generated before compilation.
add_dependencies(pick_objects_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pick_objects_server
  ${catkin_LIBRARIES}
  ${actionlib_LIBRARIES}
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES pick_objects
  CATKIN_DEPENDS roscpp visualization_msgs message_runtime actionlib move_base_msgs
  # DEPENDS system_lib
)
