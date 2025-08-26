#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double robot_x = 0.0, robot_y = 0.0;
ros::Publisher marker_pub;

double home_x = 0.0;
double home_y = 0.0;
double pickup_x = 6.0;
double pickup_y = -2.5;

const double DIST_THRESHOLD = 0.5;

enum RobotState
{
    AT_HOME,
    AT_PICKUP
};

RobotState current_state = AT_HOME;

void deleteMarker()
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_marker";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::DELETE;

    marker_pub.publish(marker);
}

void publishMarkerAt(double x, double y)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
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

    marker_pub.publish(marker);
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;

    ros::Rate r(10);

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, odometryCallback);

    ros::Duration(1.0).sleep();
    ros::spinOnce();

    double distance_to_home = hypot(robot_x - home_x, robot_y - home_y);
    if (distance_to_home < DIST_THRESHOLD)
    {
        current_state = AT_HOME;
        publishMarkerAt(pickup_x, pickup_y);
        ROS_INFO("Robot at home. Publishing pickup marker.");
    }
    else
    {
        current_state = AT_PICKUP;
        publishMarkerAt(home_x, home_y);
        ROS_INFO("Robot not at home. Publishing home marker.");
    }

    while (ros::ok())
    {
        ros::spinOnce();

        double distance_to_home = hypot(robot_x - home_x, robot_y - home_y);
        double distance_to_pickup = hypot(robot_x - pickup_x, robot_y - pickup_y);

        if (current_state == AT_HOME && distance_to_pickup < DIST_THRESHOLD)
        {
            ROS_INFO("Robot reached pickup. Waiting 5 seconds...");
            ros::Duration(5.0).sleep();

            ROS_INFO("Deleting old marker...");
            deleteMarker();
            ros::Duration(0.5).sleep();

            ROS_INFO("Publishing home marker...");
            publishMarkerAt(home_x, home_y);
            current_state = AT_PICKUP;
        }
        else if (current_state == AT_PICKUP && distance_to_home < DIST_THRESHOLD)
        {
            ROS_INFO("Robot reached home. Waiting 5 seconds...");
            ros::Duration(5.0).sleep();

            deleteMarker();
            ros::Duration(0.5).sleep();

            ROS_INFO("Publishing pickup marker...");
            publishMarkerAt(pickup_x, pickup_y);
            current_state = AT_HOME;
        }

        r.sleep();
    }

    return 0;
}
