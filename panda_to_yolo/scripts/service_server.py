#!/usr/bin/env python

from panda_to_yolo.srv import *
import rospy

def handle_add_two_ints(req):

    print ("request [%s]"%(req.flag))
    

    return pandaToYoloResponse(100,2000,40000)

def add_two_ints_server():

    rospy.init_node('panda_to_yolo_client')

    s = rospy.Service('panda_to_yolo', pandaToYolo, handle_add_two_ints)

    print( "Ready to add two ints.")

    rospy.spin()

if __name__ == "__main__":

    add_two_ints_server()