#! /usr/bin/env python
# -*- coding: utf-8 -*- 

import hsrb_interface
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox,ObjectCount

from control_msgs.msg import JointTrajectoryControllerState as JTCS

class FaceTracking:
    def __init__(self):
        self.detection_subscriber =rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.detect_face_callback)

    def detect_face_callback(self,  detection_data):
        print(detection_data.count)
        
if __name__ == '__main__':
    rospy.init_node("face_tracking")
    facetrack = FaceTracking()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()
