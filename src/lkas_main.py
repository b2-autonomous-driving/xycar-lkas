#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
# from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu


import lane_control, lane_detection, lane_estimation
from lane_control import PID_control



import sys
import os
import signal
from tf.transformations import euler_from_quaternion

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class lkas:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.init_node('xycar_lkas')
        self.image, self.imu, self.rpy =None, None, None
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback)
        self.pid = PID_control(0.5, 0.0005, 0.05)
        self.rate = rospy.Rate(30)
    
    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def imu_callback(self, data):
        self.imu = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] 
        self.rpy = euler_from_quaternion(self.imu)
        
    def run(self):

        def nothing():
            pass
        cv2.namedWindow("img")
        cv2.createTrackbar("br", "img", 0, 200, nothing)
        cv2.setTrackbarMin("br", "img", -200)
        while True:
            image_binary = lane_detection.lanedetection("gaussian_otsu").run(self.image)
            cte, curve = lane_estimation(image_binary)
            control_cmd = lane_control(cte, curve)

        rospy.spin()


if __name__ == '__main__':
    app = lkas()
    app.run()