#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Define a client for the move_base action
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variables
geometry_msgs::Pose home_pose;
bool home_pose_set = false;
MoveBaseClient *ac;
ros::Subscriber pose_sub;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if (!home_pose_set)
    {
        home_pose = msg->pose.pose;
        home_pose_set = true;
        ROS_INFO("Home position set to (x:%.2f, y:%.2f)", home_pose.position.x, home_pose.position.y);
        pose_sub.shutdown();
    }
}

void markerCallback(const visualization_msgs::Marker::ConstPtr &marker_msg)
{
    if (marker_msg->action == visualization_msgs::Marker::ADD)
    {
        ROS_INFO("Received a new marker. Setting it as the new goal.");

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = marker_msg->pose;

        ac->sendGoal(goal);
        ac->waitForResult();

        if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hooray, the robot reached the marker!");
            ROS_INFO("Returning to home position...");

            move_base_msgs::MoveBaseGoal home_goal;
            home_goal.target_pose.header.frame_id = "map";
            home_goal.target_pose.header.stamp = ros::Time::now();
            home_goal.target_pose.pose = home_pose;

            ac->sendGoal(home_goal);
            ac->waitForResult();

            if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Successfully returned to home!");
            }
            else
            {
                ROS_WARN("Failed to return to home.");
            }
        }
        else
        {
            ROS_WARN("The robot failed to reach the marker.");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_objects");
    ros::NodeHandle nh;

    // Set up the action client
    MoveBaseClient action_client("move_base", true);
    ac = &action_client;

    while (!ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Subscribe to the initial pose to set the home coordinates
    pose_sub = nh.subscribe("/amcl_pose", 1, poseCallback);

    // Subscribe to the marker topic to receive goals
    ros::Subscriber marker_sub = nh.subscribe("visualization_marker", 1, markerCallback);

    ROS_INFO("Pick objects node is running. Waiting for a marker to be published...");

    ros::spin();

    return 0;
}