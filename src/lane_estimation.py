#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu

def find_point(f_array, start, mode):
    stack = 0
    if mode == "right":
        while start < 500:
            if f_array[start] == 0:
                stack = 0
            else:
                stack += 1
                if stack >= 5:
                    return start-10
            start += 1
        return None

    if mode == "left":
        while start > 0:
            if f_array[start] == 0:
                stack = 0
            else:
                stack += 1
                if stack >= 5:
                    return start+10
            start -= 1
        return None

class estimation:

    def __init__(self):
        self.mid = 250
        self.right_point = None
        self.left_point = None
        self.prev_right_point = None
        self.prev_left_point = None

    def get_mid_poly(self, img, is_ver, is_show=False):
        f_array = img[125, :]
        self.left_point = find_point(f_array, self.mid, "left")
        if self.left_point == None:
            self.left_point = self.prev_left_point
        self.right_point = find_point(f_array, self.mid, "right")
        if self.right_point == None:
            self.right_point = self.prev_right_point
        
        self.mid = int((self.left_point + self.right_point)/2)

        if is_ver:
            if abs(self.right_point - self.left_point) < 100:
                if self.mid < 250:
                    self.mid = self.right_point - 50
                else:
                    self.mid = self.left_point + 50
        
        if is_show:
            img = cv2.line(img, (self.mid,125), (self.mid,125), 255, 5)
            cv2.imshow("gray", img)

        return -(250 - self.mid)