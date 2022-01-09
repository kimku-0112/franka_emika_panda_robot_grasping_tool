#!/usr/bin/env python3

import rospy
import cv2
import sys
from std_msgs.msg import String
from yolo_msgs.msg import yolo_pose
from yolo_msgs.srv import yolo_poses,yolo_posesResponse

import torch
import copy
from PIL import Image, ImageDraw 
import numpy as np
from models.experimental import attempt_load

class yolo_converter:

    def callback(self,data):
        capture = cv2.VideoCapture(0)
        ret, img = capture.read()
        print("SEVICE ACCPED!!")
        box_pose_list = yolo_posesResponse()
        prediction = self.model(img)
        for i, pred in enumerate(prediction):
            img = Image.fromarray(img.astype(np.uint8)) if isinstance(img, np.ndarray) else img  # from np
            if pred is not None:
                for c in pred[:, -1].unique():
                    n = (pred[:, -1] == c).sum()  # detections per class
                for *box, conf, cls in pred:  # xyxy, confidence, class
                    label = self.model.names[int(cls)] if hasattr(self.model, 'names') else 'class_%g' % cls
                    # str += '%s %.2f, ' % (label, conf)  # label
                    ImageDraw.Draw(img).rectangle(box, width=3, outline ="red")  # plot
                    open_cv_image = np.array(img) 
                    # Convert RGB to BGR 
                    open_cv_image = open_cv_image[:, :, ::-1].copy()
                    temp = []
                    for x in box:
                        temp.append(x.tolist())
                    temp_to = copy.deepcopy(self.xyxy2xywh(temp, label))
                    box_pose_list.yolo_pose.append(temp_to)
                #print(box_pose_list)
                #print("end")
            img.show()  # show
        print(box_pose_list)
        capture.release()
        return yolo_posesResponse(box_pose_list.yolo_pose)


    def __init__(self):
        self.yolo_pose = yolo_pose()
        self.model = attempt_load('/home/jo/ws_moveit/src/yolo_ros/yolov5/face.pt', map_location='cuda')
        self.model = self.model.autoshape()  # for autoshaping of PIL/cv2/np inputs and NMS
        self.yolo_server = rospy.Service("yolo_server", yolo_poses, self.callback)
        print("SERVER IS RUNNIG!!")
        print("WAITING FOR SERVICE...\n")
        # Model
        
    def xyxy2xywh(self,x, label):
        # Convert nx4 boxes from [x1, y1, x2, y2] to [x, y, w, h] where xy1=top-left, xy2=bottom-right
        self.yolo_pose.x = int((x[0] + x[2]) / 2)  # x center
        self.yolo_pose.y = int((x[1] + x[3]) / 2)  # y center
        self.yolo_pose.w = int(x[2] - x[0])  # width
        self.yolo_pose.h = int(x[3] - x[1])  # height
        self.yolo_pose.name = label
        return self.yolo_pose


def main(args):
    yc = yolo_converter()
    rospy.init_node('yolo_server', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

