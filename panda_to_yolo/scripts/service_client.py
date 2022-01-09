#!/usr/bin/env python

import sys

import rospy

from panda_to_yolo.srv import *

def add_two_ints_client():

    rospy.wait_for_service('panda_to_yolo')

    try:

        add_two_ints = rospy.ServiceProxy('panda_to_yolo', pandaToYolo )

        resp1 = add_two_ints(1)

        print ("%s + %s = %s"%(resp1.x, resp1.y, resp1.angle))


    except rospy.ServiceException as e:

        print ("Service call failed: %s"%e)

if __name__ == "__main__":
    add_two_ints_client()



