#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double robot_x, robot_y;

bool marker_visible = true;

visualization_msgs::Marker marker;

double pickup_x = 2.0;
double pickup_y = -1.0;
double dropoff_x = -3.0;
double dropoff_y = 4.0;

enum RobotState
{
    AT_HOME,
    GOING_TO_PICKUP,
    AT_PICKUP,
    GOING_TO_DROPOFF,
    AT_DROPOFF
};

RobotState current_state = AT_HOME;

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odometryCallback);

    marker.header.frame_id = "map";
    marker.ns = "pickup_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pickup_x;
    marker.pose.position.y = pickup_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    while (ros::ok())
    {
        marker.header.stamp = ros::Time::now();

        double distance_to_pickup = sqrt(pow(robot_x - pickup_x, 2) + pow(robot_y - pickup_y, 2));
        double distance_to_dropoff = sqrt(pow(robot_x - dropoff_x, 2) + pow(robot_y - dropoff_y, 2));

        if (current_state == AT_HOME)
        {
            if (distance_to_pickup < 0.5)
            {
                ROS_INFO("Robot reached the pickup zone.");
                current_state = AT_PICKUP;
                marker_visible = false;
                marker.action = visualization_msgs::Marker::DELETE;
                marker_pub.publish(marker);
                ROS_INFO("Marker for pickup zone is hidden.");
                ros::Duration(5.0).sleep(); // Simulate pickup
                ROS_INFO("Simulated pickup complete. Waiting for drop-off.");
                current_state = GOING_TO_DROPOFF;
            }
            else
            {
                marker_pub.publish(marker);
            }
        }
        else if (current_state == GOING_TO_DROPOFF)
        {
            if (distance_to_dropoff < 0.5)
            {
                ROS_INFO("Robot reached the drop-off zone.");
                current_state = AT_DROPOFF;
                marker_visible = true;
                marker.ns = "dropoff_marker";
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = dropoff_x;
                marker.pose.position.y = dropoff_y;
                marker.pose.position.z = 0;
                marker_pub.publish(marker);
                ROS_INFO("Marker for drop-off zone is shown.");
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}