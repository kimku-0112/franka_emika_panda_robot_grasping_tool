#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonTest(object):
  """ 'MoveGroupPythonTest' class """
  def __init__(self):
    super(MoveGroupPythonTest, self).__init__()
  
    # initialize 'moveit_commander' class
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    # basic info...
    eef_link = move_group.get_end_effector_link()
    # variables
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.eef_link = eef_link
  def get_current_pose(self):
    move_group = self.move_group
    current_pose = move_group.get_current_pose().pose
    return current_pose
  def go_to_pose_goal(self, pose_goal):
    move_group = self.move_group
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    return plan 

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
    scene = self.scene
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_box(self, box_name, box_pose, box_size, timeout=4):
    scene = self.scene

    scene.add_box(box_name, box_pose, box_size)

    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)
  
  def attach_box(self, box_name, timeout=4):
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link

    
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    return self.wait_for_state_update(box_name, box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, box_name, timeout=4):
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)

    return self.wait_for_state_update(box_name, box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, box_name, timeout=4):
    scene = self.scene

    scene.remove_world_object(box_name)

    return self.wait_for_state_update(box_name, box_is_attached=False, box_is_known=False, timeout=timeout)

def main():
  try:
    # initialize 'rospy' node
    rospy.init_node('move_group_python_test', anonymous=True)
    print("=========== panda moveit python test start !! ===========")

    raw_input("Initialize?, press 'Enter'...")
    test = MoveGroupPythonTest()

    raw_input("Move to Pose Goal?, press 'Enter'...")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    test.go_to_pose_goal(pose_goal)

    raw_input("Add Box?, press 'Enter'...")

    box_name = "box"
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_size = (0.1, 0.1, 0.1)
    test.add_box(box_name, box_pose, box_size)

    raw_input("Atach Box?, press 'Enter'...")
    test.attach_box(box_name)

    raw_input("Detach Box?, press 'Enter'...")
    test.detach_box(box_name)

    raw_input("Remove Box?, press 'Enter'...")
    test.remove_box(box_name)

    raw_input("Move to Pose Goal?, press 'Enter'...")
    pose_goal = test.get_current_pose()
    pose_goal.position.z += 0.2
    
    test.go_to_pose_goal(pose_goal)

    print("="*50)
    print("End Test.... Good~~~")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()






