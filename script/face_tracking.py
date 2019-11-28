#! /usr/bin/env python
# -*- coding: utf-8 -*- 

import hsrb_interface
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox

from control_msgs.msg import JointTrajectoryControllerState as JTCS


#X軸　x[pixel] = 479 * rad
#Y軸　y[pixel] = 

class FaceTracking:
    def __init__(self):
        self.detection_subscriber =rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.detect_face_callback)

        self.face_point_x = 0 #顔の中心のX座標[pixel]
        self.face_point_y = 0 #顔の中心のY座標[pixel]

        self.image_center_x = 320 #画像の中心のX座標
        self.image_center_y = 240 #画像の中心のY座標

    def detect_face_callback(self,  detection_data):

        max_prob = detection_data.bounding_boxes[0].probability #確率の最大値
        #確率の最も大きなボックスの中心座標を計算
        for i in detection_data.bounding_boxes:
            if i.probability >= max_prob:
                self.face_point_x = i.xmin + (i.xmax - i.xmin) / 2
                self.face_point_y = i.ymin + (i.ymax - i.ymin) / 2 
                print(self.face_point_x, self.face_point_y)
                max_prob = i.probability
    
    def move_hsr_head(self):'
        #X軸方向に頭を動かす角度を計算[rad]
        delta_pan_joint = (self.image_center_x - self.face_point_x) / 479.0 

		if -0.03 <= delta_pan_joint and delta_pan_joint <= 0.03:
			delta_pan_joint = 0

        whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': 0.0})


if __name__ == '__main__':
    rospy.init_node("face_tracking")
    facetrack = FaceTracking()
    rospy.spin()