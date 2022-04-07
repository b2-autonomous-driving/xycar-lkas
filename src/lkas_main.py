#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import time
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge

from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu

from lane_detection import lanedetection
from lane_control import PID_control
from lane_estimation import estimation

import sys
import os
import signal
from tf.transformations import euler_from_quaternion


def signal_handler(sig, frame):
    os.system("killall -9 python rosout")
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class lkas:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.init_node("xycar_lkas")
        self.image, self.imu, self.rpy = None, None, None
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw", Image, self.img_callback
        )
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback)
        self.pid = PID_control(0.5, 0.0, 0.05, 5)
        self.rate = rospy.Rate(30)
        self.est = estimation()

    def img_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def imu_callback(self, data):
        self.imu = [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w,
        ]
        self.rpy = euler_from_quaternion(self.imu)

    def run(self):
        while not rospy.is_shutdown():
            image_binary = lanedetection()(self.image)
            cte = self.est(image_binary)
            self.pid.drive(cte)
        self.rate.sleep()


if __name__ == "__main__":
    app = lkas()
    app.run()
