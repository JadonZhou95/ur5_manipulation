#include <cmath>
#include <list>
#include <algorithm> 

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
    bool pose_update_flag_;

    //!< Pick and place
    int pick(const geometry_msgs::Pose &pose);
    int grasp(const geometry_msgs::Pose &pose);
    void place();

    //!< Read a pose array, convert them into arm base_link and return only the top pose (z-axis)
    //!< The pose is identified only if it remains unchanged for a few readings
    int getPose(geometry_msgs::Pose &pose);
    void poseCb(const geometry_msgs::PoseArrayConstPtr p_poses);
    geometry_msgs::Pose averagePoses(const std::vector<geometry_msgs::Pose> &poses, const std::vector<int> ids, const int mean_id);

    //!< Control the gripper
    void controlGripper(const std::string &cmd, double wait_time=3.0);
    void openGripper();
    void closeGripper();
    void activateGripper();
    void pinchGripper();
    
    //!< Get Transfromation between frames
    void getTransformation(const std::string &target_frame,
                           const std::string &parent_frame,
                           geometry_msgs::TransformStamped &transform_stamped);

    //!< Get the end-effector pose to grip objects
    void transformPoseFromObjectToEE(geometry_msgs::Pose &pose_out);

    //!< Move to named pose which is defined in .srdf
    int moveToNamedPose(const std::string &name);

    //!< Move to the target pose which is relative to the /base_link
    int moveToTargetPose(const geometry_msgs::Pose &pose);

    //!< Move through waypoints along catesian paths
    int moveCatesianPath(const std::vector<geometry_msgs::Pose> &waypoints);

    int reject_pose_outliers(std::vector<geometry_msgs::Pose> &poses, std::vector<double> &norms, std::vector<int> &ids);

    template <class Type>
    void list2Vector(const std::list<Type> &l, std::vector<Type> &v);

    double getPosNorm(const geometry_msgs::Pose &pose)
    {
        return sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2) + pow(pose.position.z, 2) );
    }

public:
    PickPlacePlanner() : server_name_("[pick_place]"),
                         PLANNING_GROUP("manipulator"),
                         move_group_(PLANNING_GROUP),
                         joint_model_group_(move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP))
    {
        ROS_INFO("%s: Initializeing...", server_name_.c_str());

        // Service client to control the gripper
        gripper_client_ = nh_.serviceClient<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl>("/robotiq/control_robotiq_3f_gripper");

        // Listen to the object pose
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseArray>("/er_targets", 1,
                                                                   boost::bind(&PickPlacePlanner::poseCb, this, _1));

        // capture the transformation between frames
        this->getTransformation("ee_link", "object_link", grip2ee_tf_);
        this->getTransformation("base_link", "zed_left_camera_frame", camera2arm_tf_);
        // this->getTransformation("base_link", "camera_link", camera2arm_tf_);

        // initialize the arm and the gripper
        this->moveToNamedPose("wait");
        this->activateGripper();
        this->pinchGripper();

        ROS_INFO("%s: Ready!", server_name_.c_str());

        // main operation loop
        while(ros::ok())
        {
            bool terminate(false);

            this->moveToNamedPose("pick");
            
            geometry_msgs::Pose object_pose;
            while(this->getPose(object_pose)){}

            while(true)
            {
                if (this->pick(object_pose))
                {
                    terminate = true;    
                    break;
                }

                // ros::Duration(10).sleep();
                geometry_msgs::Pose object_pose_old = object_pose;
                if (this->getPose(object_pose))
                    break;
                
                if (fabs(object_pose_old.position.x - object_pose.position.x) < 0.05 &&
                        fabs(object_pose_old.position.y - object_pose.position.y) < 0.05 &&
                        fabs(object_pose_old.position.z - object_pose.position.z) < 0.05)
                    break;
            }

            if (!terminate && !this->grasp(object_pose))
                this->place();
        }
    }
};

int PickPlacePlanner::pick(const geometry_msgs::Pose &pose)
{
    /*
        - get pre-grasp pose
        - move to pre-grasp pose
        - move to grasp pose
        - close gripper
        - move to pre-grasp pose
        - move to wait pose
    */

    // move to pre-grasp pose
    ROS_INFO("%s: Move to Pre-Grasp Pose.", server_name_.c_str() );
    geometry_msgs::Pose pre_grasp_pose = pose;
    pre_grasp_pose.position.z += 0.05;
    this->transformPoseFromObjectToEE(pre_grasp_pose);
    if(this->moveCatesianPath(std::vector<geometry_msgs::Pose>({pre_grasp_pose})))
        return -1;

    // // move to grasp pose
    // ROS_INFO("%s: Move to Grasp Pose.", server_name_.c_str() );
    // geometry_msgs::Pose grasp_pose = pose;
    // grasp_pose.position.z -= 0.02;
    // this->transformPoseFromObjectToEE(grasp_pose);
    // this->moveCatesianPath(std::vector<geometry_msgs::Pose>({grasp_pose}));

    // // close the gripper
    // this->closeGripper();

    // // move to pose-grasp pose
    // this->moveCatesianPath(std::vector<geometry_msgs::Pose>({pre_grasp_pose}));

    // this->moveToNamedPose("wait");
    return 0;
}

int PickPlacePlanner::grasp(const geometry_msgs::Pose &pose)
{
    geometry_msgs::Pose pre_grasp_pose = pose;
    pre_grasp_pose.position.z += 0.05;
    this->transformPoseFromObjectToEE(pre_grasp_pose);

    // move to grasp pose
    ROS_INFO("%s: Move to Grasp Pose.", server_name_.c_str() );
    geometry_msgs::Pose grasp_pose = pose;
    grasp_pose.position.z -= 0.04;
    this->transformPoseFromObjectToEE(grasp_pose);
    if(this->moveCatesianPath(std::vector<geometry_msgs::Pose>({grasp_pose})))
        return -1;

    // close the gripper
    this->closeGripper();

    // move to pose-grasp pose
    if(this->moveCatesianPath(std::vector<geometry_msgs::Pose>({pre_grasp_pose})))
        return -1;

    this->moveToNamedPose("wait");
    
    return 0;
}

void PickPlacePlanner::place()
{
    this->moveToNamedPose("drop");
    this->openGripper();
    this->moveToNamedPose("wait");
}

int PickPlacePlanner::getPose(geometry_msgs::Pose &out_pose)
{
    ROS_INFO("%s: Capturing the pose...", server_name_.c_str());

    std::list<geometry_msgs::Pose> poses;
    std::list<double> norms;
    std::vector<geometry_msgs::Pose> v_poses;
    std::vector<double> v_norms;

    ros::Rate rate(10);
    double object_norm;

    double zero_time = ros::Time::now().toSec();
    double start_time = zero_time;
    while (poses.size() < 10 && ros::ok() )
    {
        // loop till updating object pose
        pose_update_flag_ = false;
        start_time = ros::Time::now().toSec();
        while(ros::ok() && !pose_update_flag_)
        {
            double current_time = ros::Time::now().toSec();
            if (current_time - start_time > 3 || current_time - zero_time > 6)
                return -1;
            rate.sleep();
        }
        
        object_norm = this->getPosNorm(object_pose_);

        poses.push_back(object_pose_);
        norms.push_back(object_norm);

        if (poses.size() > 1 && (fabs(object_norm - norms.front()) > 0.1
            || fabs(object_pose_.position.x - poses.front().position.x) > 0.1
            || fabs(object_pose_.position.y - poses.front().position.y) > 0.1
            || fabs(object_pose_.position.z - poses.front().position.z) > 0.1))
        {
            poses.pop_front();
            norms.pop_front();
        }
    }

    // for (auto it = poses.begin(); it != poses.end(); it++)
    // {
    //     std::cout << it->position.x << " "
    //               << it->position.y << " "
    //               << it->position.z << std::endl;
    // }
    // std::cout << std::endl;

    this->list2Vector(poses, v_poses);
    this->list2Vector(norms, v_norms);

    std::vector<int> ids;
    int mean_id = this->reject_pose_outliers(v_poses, v_norms, ids);
    std::cout << "mean_id: " <<  mean_id << " valid id nums:" << ids.size() << std::endl;
    
    geometry_msgs::Pose avg_pose = this->averagePoses(v_poses, ids, mean_id);

    std::cout << "Position: "
              << avg_pose.position.x << " "
              << avg_pose.position.y << " "
              << avg_pose.position.z << std::endl;

    out_pose = avg_pose;
    return 0;
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


int PickPlacePlanner::reject_pose_outliers(std::vector<geometry_msgs::Pose> &poses, std::vector<double> &norms, std::vector<int> &ids)
{
    std::vector<double> values = norms;
    std::sort(values.begin(), values.end());
    double mean = values[4];

    int mean_id(0), id(0);
    
    double std_var(0), var(0);
    for (auto it = norms.begin(); it != norms.end(); it++)
    {
        var += pow(*it - mean, 2);

        if (fabs(*it - mean) < 0.00001)
            mean_id = id;

        id++;
    }
    var /= norms.size();
    std_var = sqrt(var);

    id = 0;
    for (auto it = norms.begin(); it != norms.end(); it++)
    {
        if (fabs(mean - *it) <= 2 * std_var + 0.00001)
        {
            ids.push_back(id);
        }
        id++;
    }

    return mean_id;
}


geometry_msgs::Pose PickPlacePlanner::averagePoses(const std::vector<geometry_msgs::Pose> &poses, const std::vector<int> ids, const int mean_id)
{
    geometry_msgs::Pose avg_pose;
    avg_pose.position.x = 0;
    avg_pose.position.y = 0;
    avg_pose.position.z = 0;
    
    avg_pose.orientation = poses[mean_id].orientation;

    for (size_t i = 0; i < ids.size(); i++)
    {
        auto it = poses.begin() + ids[i];        
        avg_pose.position.x += it->position.x;
        avg_pose.position.y += it->position.y;
        avg_pose.position.z += it->position.z;
    }

    avg_pose.position.x /= ids.size();
    avg_pose.position.y /= ids.size();
    avg_pose.position.z /= ids.size();

    return avg_pose;
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

void PickPlacePlanner::transformPoseFromObjectToEE(geometry_msgs::Pose &pose)
{
    tf2::Transform base2object_tf;
    base2object_tf.setOrigin(tf2::Vector3(pose.position.x,
                                          pose.position.y,
                                          pose.position.z));
    base2object_tf.setRotation(tf2::Quaternion(pose.orientation.x,
                                               pose.orientation.y,
                                               pose.orientation.z,
                                               pose.orientation.w));
    
    // grip2ee_tf_
    tf2::Transform object2ee_tf;
    object2ee_tf.setOrigin(tf2::Vector3(grip2ee_tf_.transform.translation.x,
                                        grip2ee_tf_.transform.translation.y,
                                        grip2ee_tf_.transform.translation.z));

    object2ee_tf.setRotation(tf2::Quaternion(grip2ee_tf_.transform.rotation.x,
                                             grip2ee_tf_.transform.rotation.y,
                                             grip2ee_tf_.transform.rotation.z,
                                             grip2ee_tf_.transform.rotation.w));

    tf2::Transform base2ee_tf = base2object_tf * object2ee_tf.inverse();
    tf2::Vector3 vec = base2ee_tf.getOrigin();
    tf2::Quaternion quat = base2ee_tf.getRotation();

    pose.position.x = vec.x();
    pose.position.y = vec.y();
    pose.position.z = vec.z();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

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
        moveit_msgs::MoveItErrorCodes error_code = move_group_.move();
        if (error_code.val == error_code.SUCCESS)
            return 0;
    }
    
    ROS_ERROR("%s: Fail to move arm to Pose %s", server_name_.c_str(), name.c_str());
    return -1;
}


int PickPlacePlanner::moveToTargetPose(const geometry_msgs::Pose &pose)
{
    move_group_.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group_.move();
        return 0;
    }
    else
    {
        ROS_ERROR("%s: Fail to move arm to target pose", server_name_.c_str() );
        return -1;
    }
}


int PickPlacePlanner::moveCatesianPath(const std::vector<geometry_msgs::Pose> &waypoints)
{
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9)
    {
        moveit_msgs::MoveItErrorCodes error_code = move_group_.execute(trajectory);
        if (error_code.val == error_code.SUCCESS)
            return 0;
    }
    ROS_WARN("%s: Only %.2f of the catesian path can be completed.", server_name_.c_str(), fraction);
    return -1;
}

void PickPlacePlanner::controlGripper(const std::string &cmd, double wait_time)
{
    robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl ser;
    ser.request.command = cmd;
    gripper_client_.call(ser);

    ros::Duration(wait_time).sleep();
    return;
}

void PickPlacePlanner::openGripper()
{
    ROS_INFO("Open Gripper");
    this->controlGripper("o");
}

void PickPlacePlanner::closeGripper()
{
    ROS_INFO("Close Gripper");
    this->controlGripper("c");
}

void PickPlacePlanner::activateGripper()
{
    ROS_INFO("Open Gripper");
    this->controlGripper("a", 20);
}

void PickPlacePlanner::pinchGripper()
{
    ROS_INFO("Set Gripper in Mode Pinch");
    this->controlGripper("p");
}

template <class Type>
void PickPlacePlanner::list2Vector(const std::list<Type> &l, std::vector<Type> &v)
{
    for (auto it = l.begin(); it != l.end(); it++)
        v.push_back(*it);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_pick_and_place_node");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    PickPlacePlanner pickPlacePlanner;

    return 0;
}
