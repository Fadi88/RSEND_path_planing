#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/UpdateMarker.h"
#include "add_markers/DeleteMarkers.h"

ros::Publisher marker_pub;

bool handle_update_marker(add_markers::UpdateMarker::Request &req,
                          add_markers::UpdateMarker::Response &res)
{
    ROS_INFO("Service called to update marker to x=%.2f, y=%.2f", req.x, req.y);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = req.x;
    marker.pose.position.y = req.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);

    res.success = true;
    return true;
}

bool handle_delete_marker(add_markers::DeleteMarkers::Request &req,
                          add_markers::DeleteMarkers::Response &res)
{
    visualization_msgs::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = ros::Time::now();
    delete_marker.ns = "goal_marker";
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_pub.publish(delete_marker);

    res.success = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_marker_server");
    ros::NodeHandle n;

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::ServiceServer service_update = n.advertiseService("update_marker", handle_update_marker);
    ros::ServiceServer service_delete = n.advertiseService("delete_marker", handle_delete_marker);

    ros::spin();

    return 0;
}
