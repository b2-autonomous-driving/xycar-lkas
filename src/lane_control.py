#!/usr/bin/env python

import rospy, rospkg
import numpy as np
from geometry_msgs.msg import Point
from xycar_msgs.msg import xycar_motor
from math import atan2, sin

import sys
import os
import signal

class PID_control:

    def __init__(self, kp, ki, kd, speed):

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.speed = speed
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    def drive(self, cte):
        Angle = self.pid_control(cte)
        msg = xycar_motor()
        msg.angle = Angle
        msg.speed = self.speed
        self.pub.publish(msg)

    def pid_control(self, error):
        self.d_error = error - self.p_error
        self.p_error = error
        self.i_error += error

        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error

class Pure_pursuit:

    def __init__(self):

        self.lfd = 3
        self.point = Point()
        self.is_point = False
        rate = rospy.Rate(30)

        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.sub = rospy.Subscriber("/offset", Float32, self.offset_callback)

        while not rospy.is_shutdown():
            if is_offset:
                points = self.points_to_path(self.point)
                theta = atan2(points[0], points[1])
                steering = -atan2((2*sin(theta)), self.lfd)
                self.drive(steering, 10)
                rate.sleep()

    def point_callback(self, data):
        self.is_point = True
        self.point.x = data.x
        self.point.y = data.y
        self.point.z = 0

    def drive(self, Angle, Speed):
        msg = xycar_motor()
        msg.angle = Angle
        msg.speed = Speed
        self.pub.publish(msg)

    def points_to_path(self, points):
        point = [0,0]
        point[0] = (points.x - 320)*2.5/420
        point[1] = points.y*5/400 + 2

        return point



