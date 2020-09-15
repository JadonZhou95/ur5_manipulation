#include "ros/ros.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotControl.h"

class PickPlacePlanner
{
    private:
        ros::NodeHandel nh_;

        const std::string PLANNING_GROUP;
        moveit::planning_interface::MoveGroupInterface move_group_;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        const moveit::core::JointModelGroup* joint_model_group_;

        ros::ServiceClient gripper_client_;

        void openGripper();
        void closeGripper();
    
    public:
        PickPlaceMotion():
        PLANNING_GROUP("manipulator"),
        move_group_(PLANNING_GROUP),
        joint_model_group_(move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP))
        {
            gripper_client_ = nh_.serviceClient<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl>("/robotiq/control_robotiq_3f_gripper");

        }





}

void PickPlacePlanner::openGripper()
{
    robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl ser;
    ser.request.command = "o";
    gripper_client.call(ser);

    ros::Duration(3.0).sleep();
    return;
}

void PickPlacePlanner::closeGripper()
{
    robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl ser;
    ser.request.command = "c";
    gripper_client.call(ser);

    ros::Duration(3.0).sleep();
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_pick_and_place_node");

    PickPlacePlanner pickPlacePlanner();



    
}

