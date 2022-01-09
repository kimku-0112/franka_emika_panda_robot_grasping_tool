#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "my_cobot/goal_pose.h"
#include "my_cobot/goal_joint.h"
//#include "my_cobot/set_path.h"
//#include "my_cobot/move_path.h"


const double tau = 2 * M_PI;
moveit::planning_interface::MoveGroupInterface *move_group;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
std::vector<double> joint_positions;
//x, y, z =
bool goalPoseCallback(my_cobot::goal_pose::Request &req, my_cobot::goal_pose::Response &res){
    geometry_msgs::Pose pose;
    tf2::Quaternion orientation;
    orientation.setRPY(req.rpy.x, req.rpy.y, req.rpy.z);
    pose.orientation = tf2::toMsg(orientation);
    pose.position = req.point;
    
    move_group->setPoseTarget(pose);
    
    res.success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(res.success){
        move_group->move();
        return true;
    }
    else{
        ROS_ERROR("Cannot move to Position!");
        return false;
    }
}

bool goalJointCallback(my_cobot::goal_joint::Request &req, my_cobot::goal_joint::Response &res){
    if(req.joint.size() != 7){
        ROS_ERROR("Insufficient number of joints!");
        return false; 
    }
    else{
        for(int i{}; i < 7; i++){
            ROS_INFO("joint %d :: %lf",i , req.joint[i]);
            joint_positions.push_back(req.joint[i]);
        }
    }
    move_group->setJointValueTarget(joint_positions);
    res.success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(res.success){
        move_group->move();
        joint_positions.clear();
        return true;
    }
    else{
        ROS_ERROR("Cannot move to Position!");
        joint_positions.clear();
        return false;
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "cobot_node");
    ros::NodeHandle nh;
    ros::ServiceServer pose_server = nh.advertiseService("pose_server", goalPoseCallback);
    ros::ServiceServer joint_server = nh.advertiseService("joint_server", goalJointCallback);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP = "panda_arm";
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO("check");
    

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    //visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Hellow Cobot Wolrd", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group->getPlanningFrame().c_str());

    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group->getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group->getJointModelGroupNames().begin(), move_group->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
   
    return 0;
}