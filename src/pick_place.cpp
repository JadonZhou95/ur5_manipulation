#include "ros/ros.h"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotControl.h"

class PickPlacePlanner
{
    private:
        ros::NodeHandle nh_;

        const std::string server_name_;

        const std::string PLANNING_GROUP;
        moveit::planning_interface::MoveGroupInterface move_group_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        const moveit::core::JointModelGroup* joint_model_group_;

        ros::ServiceClient gripper_client_; //!< Clinet to call the robotiq gripper
        
        //!< Contro the gripper
        void openGripper();
        void closeGripper();
    
    public:
        PickPlacePlanner():
            server_name_("[pick_place]"),
            PLANNING_GROUP("manipulator"),
            move_group_(PLANNING_GROUP),
            joint_model_group_(move_group_.getCurrentState()->getJointModelGroup(PLANNING_GROUP))
        {
            ROS_INFO("%s: Initializeing...", server_name_.c_str());
            std::cout << "12345" << std::endl;
            gripper_client_ = nh_.serviceClient<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotControl>("/robotiq/control_robotiq_3f_gripper");

            std::vector<std::string> named_pose = {"wait", "pick", "wait", "drop", "wait"};
            for (size_t i = 0; i < named_pose.size(); i++)
            {
                std::map<std::string, double> joint_values = move_group_.getNamedTargetValues(named_pose[i]);
                move_group_.setJointValueTarget(joint_values);

                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                bool success = (move_group_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if (success)
                    move_group_.move();
                
            }
        }

};

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_pick_and_place_node");
    ros::AsyncSpinner spinner(1); spinner.start();
    std::cout << "Hello world!" << std::endl;

    PickPlacePlanner pickPlacePlanner;

    std::cout << "end" << std::endl;
    return 0;
    
}

