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
    def __init__(self, kp, ki, kd, speed=10):

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.speed = speed
        self.striaghtSpeed = speed
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

        self.pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)

    def drive(self, cte):
        Angle = self.pid_control(cte)
        msg = xycar_motor()
        msg.angle = Angle
        msg.speed = self.striaghtSpeed if abs(Angle) < 30 else 5
        self.pub.publish(msg)

    def pid_control(self, error):
        self.d_error = error - self.p_error
        self.p_error = error
        self.i_error += error

        return self.Kp * self.p_error + self.Ki * self.i_error + self.Kd * self.d_error
