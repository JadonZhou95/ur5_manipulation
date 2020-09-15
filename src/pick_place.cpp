#include <cmath>

#include "ros/ros.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotControl.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"

// #include "tf/tf.h"

class PickPlacePlanner
{
private:
    ros::NodeHandle nh_;

    const std::string server_name_;

    const std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    const moveit::core::JointModelGroup *joint_model_group_;

    ros::ServiceClient gripper_client_; //!< Clinet to call the robotiq gripper
    ros::Subscriber pose_sub_;          //!< Subscriber to get the object pose

    geometry_msgs::TransformStamped camera2arm_tf_,
        grip2ee_tf_;

    geometry_msgs::Pose object_pose_;
    short min_pose_read_count_;
    bool pose_update_flag_;

    //!< Pick and place
    void pick(const geometry_msgs::Pose &pose);
    void place();

    //!< Read a pose array, convert them into arm base_link and return only the top pose (z-axis)
    //!< The pose is identified only if it remains unchanged for a few readings
    geometry_msgs::Pose getPose();
    void poseCb(const geometry_msgs::PoseArrayConstPtr p_poses);

    //!< Control the gripper
    void openGripper();
    void closeGripper();

    //!< Get Transfromation between frames
    void getTransformation(const std::string &target_frame,
                           const std::string &parent_frame,
                           geometry_msgs::TransformStamped &transform_stamped);

    int moveToNamedPose(const std::string &name);

public:
    PickPlacePlanner() : server_name_("[pick_place]"),
                         PLANNING_GROUP("manipulator"),
                         move_group_(PLANNING_GROUP),
                         joint_model_group_(move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP))
    {
        ROS_INFO("%s: Initializeing...", server_name_.c_str());

        gripper_client_ = nh_.serviceClient<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl>("/robotiq/control_robotiq_3f_gripper");

        pose_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/er_targets", 1,
                                                                    boost::bind(&PickPlacePlanner::poseCb, this, _1));

        this->moveToNamedPose("wait");

        // capture the transformation between frames
        this->getTransformation("base_link", "camera_link", camera2arm_tf_);
        this->getTransformation("grip_point", "ee_link", grip2ee_tf_);

        min_pose_read_count_ = 5;
        pose_update_flag_ = false;

        ROS_INFO("%s: Ready!", server_name_.c_str());

        while(ros::ok())
        {
            geometry_msgs::Pose object_pose = this->getPose();
            this->pick(object_pose);
            this->place();
        }

    }
};

void PickPlacePlanner::pick(const geometry_msgs::Pose &pose)
{
    this->moveToNamedPose("pick");

    /*
        - get pre-grasp pose
        - move to pre-grasp pose
        - move to grasp pose
        - close gripper
        - move to pre-grasp pose
        - move to wait pose
    */
}

void PickPlacePlanner::place()
{
    this->moveToNamedPose("drop");
    this->openGripper();
    this->moveToNamedPose("wait");
}

geometry_msgs::Pose PickPlacePlanner::getPose()
{
    ROS_INFO("%s: Capturing the pose...", server_name_.c_str());

    ros::Rate rate(10);
    short pose_read_count = 0;
    geometry_msgs::Pose object_pose_tmp = object_pose_;

    while (pose_read_count < min_pose_read_count_ && ros::ok())
    {
        if (pose_update_flag_)
        {
            if (fabs(object_pose_.position.x - object_pose_tmp.position.x) < 0.01 &&
                fabs(object_pose_.position.y - object_pose_tmp.position.y) < 0.01 &&
                fabs(object_pose_.position.z - object_pose_tmp.position.z) < 0.01)
            {
                pose_read_count++;
            }
            else
            {
                object_pose_tmp = object_pose_;
                pose_read_count = 0;
            }
        }

        rate.sleep();
        pose_update_flag_ = false;
    }

    return object_pose_;
}

void PickPlacePlanner::poseCb(const geometry_msgs::PoseArrayConstPtr p_poses)
{
    if (p_poses->poses.size() == 0)
        return;

    // point with pose with max z value in arm
    geometry_msgs::Pose top_pose;
    top_pose.position.z = -100; // inti with a small dummy value

    for (size_t i = 0; i < p_poses->poses.size(); i++)
    {
        // Transform from camera frame to ur base frame
        geometry_msgs::Pose pose_tr;
        tf2::doTransform(p_poses->poses[i], pose_tr, camera2arm_tf_);

        if (pose_tr.position.z > top_pose.position.z)
            top_pose = pose_tr;
    }

    object_pose_ = top_pose;
    pose_update_flag_ = true;
}

void PickPlacePlanner::getTransformation(const std::string &target_frame,
                                         const std::string &parent_frame,
                                         geometry_msgs::TransformStamped &transform_stamped)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while (ros::ok())
    {
        try
        {
            transform_stamped = tfBuffer.lookupTransform(target_frame, parent_frame,
                                                         ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Failed to capture the transform with error %s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    return;
}

int PickPlacePlanner::moveToNamedPose(const std::string &name)
{
    // std::vector<std::string> named_pose = {"wait", "pick", "wait", "drop", "wait"};
    ROS_INFO("%s: Move arm to Pose %s", server_name_.c_str(), name.c_str());
    std::map<std::string, double> joint_values = move_group_.getNamedTargetValues(name);
    move_group_.setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_.move();
        return 0;
    }
    else
    {
        ROS_ERROR("%s: Fail to move arm to Pose %s", server_name_.c_str(), name.c_str());
        return -1;
    }
}

void PickPlacePlanner::openGripper()
{
    robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl ser;
    ser.request.command = "o";
    gripper_client_.call(ser);

    ros::Duration(3.0).sleep();
    return;
}

void PickPlacePlanner::closeGripper()
{
    robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl ser;
    ser.request.command = "c";
    gripper_client_.call(ser);

    ros::Duration(3.0).sleep();
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_pick_and_place_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    PickPlacePlanner pickPlacePlanner;

    return 0;
}
