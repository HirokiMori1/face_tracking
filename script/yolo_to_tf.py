#! /usr/bin/env python
# -*- coding: utf-8 -*-    

import rospy
import cv2
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import image_geometry
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import tf2_ros




class Sample:
    def __init__(self):
        self.pinhole = image_geometry.PinholeCameraModel()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.bridge = CvBridge()
        cam_info = rospy.wait_for_message('/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo)
        self.pinhole.fromCameraInfo(cam_info)

        depth_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image', Image)
        image_sub = message_filters.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image)
        ts = message_filters.ApproximateTimeSynchronizer([depth_sub , image_sub], 10 , 0.3,allow_headerless=True)

        ts.registerCallback(self.calc_distance)

    def calc_distance(self, depth_data, image_data):
        depth = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')
        img = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        #TODO:
        y = image_data.height/2
        x = image_data.width/2

        print(x,y)

        ray = self.pinhole.projectPixelTo3dRay([x, y])
        camera_point = np.array(ray) * depth[y, x]
        print(camera_point)

        cam_point = PointStamped()
        cam_point.header.frame_id = image_data.header.frame_id
        cam_point.point.x = camera_point[0]
        cam_point.point.y = camera_point[1]
        cam_point.point.z = camera_point[2]

        base_stamp = self.tfBuffer.transform(cam_point, "base_link", timeout=rospy.Duration(5))
        #print(base_stamp)


if __name__ == "__main__":
    rospy.init_node("calc_obj_distance")
    print("aa")
    sample = Sample()
    rospy.spin()
    
