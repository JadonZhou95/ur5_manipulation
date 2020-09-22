#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "posearray_publish_node");
    ros::NodeHandle nh;

    geometry_msgs::PoseArray pose_array;
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = "camera_link";

    geometry_msgs::Pose pose;
    pose.position.x = 0.6;
    pose.position.y = -0.3;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    pose_array.poses.push_back(pose);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("er_targets", 1); 

    ros::Rate rate(30);
    while (ros::ok())
    {
        pub.publish(pose_array);
        rate.sleep();
    }

    return 0;
}