#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"

geometry_msgs::Pose pose0;

void posCb(const geometry_msgs::PoseConstPtr&msg)
{
    pose0.position = msg->position;
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "posearray_publish_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("er_targets", 1); 
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Pose>("pose_sample", 1, boost::bind(posCb, _1));

    pose0.position.x = 0.6;
    pose0.position.y = 0.0;
    pose0.position.z = 0.0;
    pose0.orientation.x = 0;
    pose0.orientation.y = 0;
    pose0.orientation.z = 0;
    pose0.orientation.w = 1;

    ros::Rate rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        
        geometry_msgs::PoseArray pose_array;
        pose_array.header.stamp = ros::Time::now();
        pose_array.header.frame_id = "camera_link";
        pose_array.poses.push_back(pose0);
        
        pub.publish(pose_array);
        rate.sleep();
    }

    return 0;
}