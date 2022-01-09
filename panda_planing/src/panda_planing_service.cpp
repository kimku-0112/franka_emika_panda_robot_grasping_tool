#include "ros/ros.h"                          
#include "panda_planing/goal_joint.h"
#include "panda_planing/goal_pose.h"
#include "panda_planing/grasp.h"
#include "panda_planing/pandaToYolo.h"

#include <unistd.h>
#include <cstdlib>       

#define Z_UP 0.087
#define Z 0.038
#define BOX_Z 0.25

  ros::ServiceClient pose_client;
  ros::ServiceClient joint_client;
  ros::ServiceClient gripper_client;
  ros::ServiceClient yolo_client;

  long x,y,object;
  double angle;

bool goto_target_joint(double * joint_arr){
  panda_planing::goal_joint joint_srv;
  joint_srv.request.joint.resize(7);
  for(int i = 0; i < 7; i++) joint_srv.request.joint[i] = joint_arr[i];

  if (joint_client.call(joint_srv))
  {    

    for(int i = 0; i < 7; i++) ROS_INFO("send msg %lf", joint_srv.request.joint[i]);
    ROS_INFO("receive srv, srv.Response.result: %ld", (long int)joint_srv.response.success);
    return 1;

  }
  else
  {
    ROS_ERROR("Failed to call service joint");
    return 0;
  }

}

bool goto_target_pose(double x, double y, double z, double r, double p,double Y){
  panda_planing::goal_pose pose_srv;
  pose_srv.request.point.x = x;
  pose_srv.request.point.y = y;
  pose_srv.request.point.z = z;
  pose_srv.request.rpy.x = r;
  pose_srv.request.rpy.y = p;
  pose_srv.request.rpy.z = Y;
  if (pose_client.call(pose_srv))
  {
    ROS_INFO("receive srv, srv.Response.result: %ld", (long int)pose_srv.response.success);
    return 1;
  }
  else
  {
    ROS_ERROR("Failed to call service pose");
    return 0;
  }

}

bool goto_target_grasp(bool close){
  sleep(0.1);
  panda_planing::grasp grasp_srv;
  if(close){
    grasp_srv.request.release = false;
    grasp_srv.request.move = true;
    grasp_srv.request.homing = false;
    grasp_srv.request.power = 50;
    grasp_srv.request.grasp_width = 0;
    grasp_srv.request.speed = 0.1;
  }
  else{
    grasp_srv.request.release = false;
    grasp_srv.request.move = false;
    grasp_srv.request.homing = true;
    grasp_srv.request.power = 0.5;
    grasp_srv.request.grasp_width = 0;
    grasp_srv.request.speed = 0;
  }
  if (gripper_client.call(grasp_srv))
  {
    ROS_INFO("receive srv, srv.Response.result: %ld", (long int)grasp_srv.response.success);
    return 1;
  }
  else
  {
    ROS_ERROR("Failed to call service gripper");
    return  goto_target_grasp(close);
  }
}

void get_target_pose(bool get){
  panda_planing::pandaToYolo Yolo_srv;
  Yolo_srv.request.flag = get;
  if (yolo_client.call(Yolo_srv))
  {
    x = Yolo_srv.response.x;
    y = Yolo_srv.response.y;
    object = Yolo_srv.response.object;
    angle = Yolo_srv.response.angle;
    ROS_INFO("receive srv, srv.Response.result: %ld  %ld  %lf %ld",x,y,angle,object);
  }
  else
  {
    ROS_ERROR("Failed to   service yolo");
  }

}

int main(int argc, char **argv)              
{
  ros::init(argc, argv, "service_client");   

  ros::NodeHandle nh;    
  joint_client = nh.serviceClient<panda_planing::goal_joint>("joint_server");
  pose_client = nh.serviceClient<panda_planing::goal_pose>("pose_server");
  gripper_client = nh.serviceClient<panda_planing::grasp>("gripper_server");
  yolo_client = nh.serviceClient<panda_planing::pandaToYolo>("panda_to_yolo");

  double joint_arr[7] = {0.026, -0.126, 0.003, -2.203, 0.002, 2.081, 0.805};
  
  goto_target_grasp(0);
  while(1){
    goto_target_joint(joint_arr);
    sleep(0.1);
    get_target_pose(0); // buffer clear
    sleep(0.2);
    get_target_pose(1); // get data
    ROS_INFO("receive srv, srv.Response.result: %ld  %ld  %lf %ld",x, y, angle , object);
    if(object == 99){

    }
    else{
      float m_x = 0.75 - y * 0.0008;
      float m_y = 0.32 - x * 0.0008;
      if(angle > 3.1)angle = 3.1;
      else if(angle < -3.1) angle = -3.1;
      float m_a =  angle + 2.34;

      goto_target_pose(m_x,m_y,Z_UP,0.0,3.14,m_a);
      goto_target_pose(m_x,m_y,Z,0.0,3.14,m_a);
      goto_target_grasp(0);
      goto_target_grasp(1);
      goto_target_pose(m_x,m_y,Z_UP,0.0,3.14,m_a);
      goto_target_joint(joint_arr);
      goto_target_pose(0.0,0.3,0.5,0.0,3.14,3.92);

      if(object == 1 || object == 13) { // nipper
        goto_target_pose(0.035,0.75,BOX_Z,0.0,3.14,3.92);
        goto_target_grasp(0);
        goto_target_pose(0.035,0.75,2.0,0.0,3.14,3.92);
      }
      else if(object == 0 || object == 11) { // driver
        goto_target_pose(0.035,0.55,BOX_Z,0.0,3.14,3.92);
        goto_target_grasp(0);
        goto_target_pose(0.035,0.55,2.0,0.0,3.14,3.92);
      }
      else if(object == 5 || object == 7 || object == 9) { // knife
        goto_target_pose(-0.19,0.75,BOX_Z,0.0,3.14,3.92);
        goto_target_grasp(0);        
        goto_target_pose(-0.19,0.75,2.0,0.0,3.14,3.92);
      }
      else if(object == 4 || object == 15) { 
        goto_target_pose(-0.19,0.55,BOX_Z,0.0,3.14,3.92);
        goto_target_grasp(0);
        goto_target_pose(-0.19,0.55,2.0,0.0,3.14,3.92);
      }

      //goto_target_grasp(1);
      //goto_target_pose(0.0,0.3,0.5,0.0,3.14,-0.8);
      //goto_target_grasp(0);
    }
  }

  return 0;
}
