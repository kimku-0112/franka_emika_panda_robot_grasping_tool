#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>

#include <ros/ros.h>

#include "ros/ros.h"
#include "my_cobot/grasp.h"

franka::Gripper *gripper;
franka::GripperState gripper_state;

bool gripperCallback(my_cobot::grasp::Request &req, my_cobot::grasp::Response &res){
    try{
        if(req.release){ 
            gripper->stop();
        }
        else if(req.homing){
            gripper->homing();
        }
        else if(req.move){
            gripper_state = gripper->readOnce();
            if (gripper_state.max_width < req.grasp_width) {
                std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
                res.success = false;
                return -1;
            }
            if (!gripper->move(req.grasp_width, req.speed)) {
                std::cout << "Failed to move." << std::endl;
                res.success = false;
                return -1;
            }
        }
        else{
            gripper_state = gripper->readOnce();
            if (gripper_state.max_width < req.grasp_width) {
                std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
                res.success = false;
                return -1;
            }
            if (!gripper->grasp(req.grasp_width, req.speed, req.power)) {
                std::cout << "Failed to grasp object." << std::endl;
                res.success = false;
                return -1;
            }
            std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));
            gripper_state = gripper->readOnce();
            if (!gripper_state.is_grasped) {
                std::cout << "Object lost." << std::endl;
                res.success = false;
                return -1;
            }
        }
        res.success = true;
        return true;
    }
    catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1; 
    }
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "gripper_node");
    ros::NodeHandle nh;
    ros::ServiceServer gripper_server = nh.advertiseService("gripper_server", gripperCallback);

    std::string robot_ip = "192.168.1.151";

    ROS_INFO("nice start"); 

    ROS_INFO("nice   %s",robot_ip.c_str()); 
    
    if(ros::param::has("~robot_ip")){
        ros::param::get("~robot_ip",robot_ip);
    }

    ROS_INFO("%s",robot_ip.c_str());    
    double default_speed(0.1);

    gripper = new franka::Gripper(robot_ip);

    ros::spin();

    return 0;
}