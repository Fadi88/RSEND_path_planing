#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "pick_objects/SetAndNavigate.h"
#include "pick_objects/ReturnHome.h"

// Global variables to store the latest marker pose and a flag indicating if a marker has been received.
geometry_msgs::PoseStamped latest_marker_pose;
bool marker_received = false;

// Typedef for the action client to make the code more readable.
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void markerCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    if (msg->action == visualization_msgs::Marker::ADD)
    {
        latest_marker_pose.header = msg->header;
        latest_marker_pose.pose = msg->pose;
        marker_received = true;
    }
}

bool handle_set_and_navigate(pick_objects::SetAndNavigate::Request &req,
                             pick_objects::SetAndNavigate::Response &res)
{
    if (!marker_received)
    {
        ROS_WARN("No marker pose received. Cannot send navigation goal.");
        res.success = false;
        return true;
    }

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = latest_marker_pose;

    ac.sendGoal(goal);

    ROS_INFO("Navigation goal sent to x=%.2f, y=%.2f.", latest_marker_pose.pose.position.x, latest_marker_pose.pose.position.y);
    res.success = true;
    ac.waitForResult();

    return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool handle_return_home(pick_objects::ReturnHome::Request &req,
                        pick_objects::ReturnHome::Response &res)
{
    ROS_INFO("Returning home...");

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    ac.sendGoal(goal);
    ac.waitForResult();

    // Set the response based on the action result.
    bool success = ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    res.success = success;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_objects_server");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("visualization_marker", 1, markerCallback);

    ros::ServiceServer service_1 = n.advertiseService("set_and_navigate", handle_set_and_navigate);
    ros::ServiceServer service_2 = n.advertiseService("return_home", handle_return_home);

    ROS_INFO("Marker navigation server ready. Call the /set_and_navigate service or /return_home to begin.");

    ros::spin();

    return 0;
}
