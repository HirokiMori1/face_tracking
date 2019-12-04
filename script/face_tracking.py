#! /usr/bin/env python
# -*- coding: utf-8 -*- 

import hsrb_interface
import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox,ObjectCount

from control_msgs.msg import JointTrajectoryControllerState as JTCS


#X軸の角度の変化とpixelの変化の関数　x[pixel] = 479 * rad
#Y軸の角度の変化とpixelの変化の関数　y[pixel] = 475 * rad

robot = hsrb_interface.Robot()
whole_body = robot.get('whole_body')

#HSRの頭の可動範囲
min_pan_joint = -3.8
max_pan_joint = 1.7

min_tilt_joint = -1.5
max_tilt_joint = 0.52

class FaceTracking:
    def __init__(self):
        self.detection_subscriber =rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.detect_face_callback)
        self.foundobject_subscriber = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.detect_object)
        #顔の位置のPublisher
        self.point_pub = rospy.Publisher('face_point', Point, queue_size=10)

        self.face_point_x = 0 #顔の中心のX座標[pixel]
        self.face_point_y = 0 #顔の中心のY座標[pixel]

        self.image_center_x = 320 #画像の中心のX座標
        self.image_center_y = 240 #画像の中心のY座標

        self.is_detect_object = 0 #オブジェクト検出フラグ

    def detect_face_callback(self,  detection_data):

        point_x = 0.0
        point_y = 0.0

        max_prob = 0.0 #確率の最大値
        #確率の最も大きなボックスの中心座標を計算
        for i in detection_data.bounding_boxes:
            if i.probability >= max_prob:
                point_x = i.xmin + (i.xmax - i.xmin) / 2
                point_y = i.ymin + (i.ymax - i.ymin) / 2 
                max_prob = i.probability

        self.face_point_x = point_x
        self.face_point_y = point_y

        face_point = Point()

        face_point.x = point_x
        face_point.y = point_y
        face_point.z = 0
        #顔の位置をpublish
        self.point_pub.publish(face_point)

    #オブジェクトを検出しているかを判定
    def detect_object(self, object_data):
        self.is_detect_object = 0 if object_data.count == 0 else 1 #オブジェクト検出数が0ならオブジェクト検出フラグを0にする
        #print(self.is_detect_object)

    
    def move_hsr_head(self):
        #X軸方向に頭を動かす角度を計算[rad]
        delta_pan_joint = (self.image_center_x - self.face_point_x) / 479.0 
        #Y軸方向に頭を動かす角度を計算[rad]
        delta_tilt_joint = (self.image_center_y - self.face_point_y) / 475.0

        if -0.03 <= delta_pan_joint and delta_pan_joint <= 0.03:
			delta_pan_joint = 0.0
        if -0.03 <= delta_tilt_joint and delta_tilt_joint <= 0.03:
            delta_tilt_joint = 0.0
            
        #現在のheadの角度を取得
        #joint_names: [head_tilt_joint, head_pan_joint]
        current_head_joint = rospy.wait_for_message("/hsrb/head_trajectory_controller/state", JTCS, timeout=None)
        current_pan_joint = current_head_joint.desired.positions[1] #[rad]
        current_tilt_joint = current_head_joint.desired.positions[0] #[rad]

        #頭を動かす角度の計算[rad]
        move_pan_joint = current_pan_joint + delta_pan_joint
        move_tilt_joint = current_tilt_joint + delta_tilt_joint

        #頭を動かす角度を制限
        if move_pan_joint <= min_pan_joint:
            move_pan_joint = min_pan_joint
        elif move_pan_joint >= max_pan_joint:
            move_pan_joint = max_pan_joint

        if move_tilt_joint < min_tilt_joint:
            move_tilt_joint = min_tilt_joint
        elif move_tilt_joint >= max_tilt_joint:
            move_tilt_joint = max_tilt_joint

        if self.is_detect_object != 0:
            whole_body.move_to_joint_positions({'head_pan_joint': move_pan_joint, 'head_tilt_joint': move_tilt_joint})
        elif self.is_detect_object == 0: #検出している物体がなかったら原点に戻す
            whole_body.move_to_joint_positions({'head_pan_joint': 0.0, 'head_tilt_joint': 0.0})

if __name__ == '__main__':
    #rospy.init_node("face_tracking")
    facetrack = FaceTracking()

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        facetrack.move_hsr_head()
        rate.sleep()

    rospy.spin()