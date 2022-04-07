#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class videoreader:
    def __init__(
        self, file_path="/home/killywonka/xycar_ws/src/xycar_lkas/src/track2.avi"
    ):
        self.bridge = CvBridge()
        self.file_path = file_path
        self.cap = cv2.VideoCapture(self.file_path)
        rospy.init_node("video_reader")
        self.pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        self.msg = Image()
        self.image = None
        self.rate = rospy.Rate(30)
        self.play = False

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                break
            cur_frame = self.cap.get(cv2.CAP_PROP_POS_FRAMES)
            cv2.imshow("sender", frame)
            wk = cv2.waitKey(1)

            if not self.play:
                cur_frame -= 1

            if wk == ord("a"):
                cur_frame -= 1
            if wk == ord("d"):
                cur_frame += 1
            if wk == 32:
                self.play = not self.play

            self.cap.set(cv2.CAP_PROP_POS_FRAMES, cur_frame)

            self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.rate.sleep()


app = videoreader()
app.run()
