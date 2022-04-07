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

        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    

    def drive(self, cte):
        Angle = self.pid_control(cte)
        msg = xycar_motor()
        msg.angle = Angle
        msg.speed = self.striaghtSpeed if Angle < abs(30) else 5
        self.pub.publish(msg)

    def pid_control(self, error):
        self.d_error = error - self.p_error
        self.p_error = error
        self.i_error += error

        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error

# class Pure_pursuit:

#     def __init__(self, lfd, x_ratio, y_ratio):

#         self.lfd = lfd
#         self.x_ratio = x_ratio
#         self.y_ratio = y_ratio

#         self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)


#         while not rospy.is_shutdown():
#             if is_offset:
#                 points = self.points_to_path(self.point)
                
#                 self.drive(steering, 10)
#                 rate.sleep()

#     def adjust_values(pid, angle):
#         if angle> 30 or angle < -30: 
#             pid.speed = 5
#         else: 
#             pid.speed = 15

#     def drive(self, cte, Speed):

#         x = cte / self.x_ratio
#         y = 240 / self.y_ratio

#         theta = atan2(points[0], points[1])
#         steering = -atan2((2*sin(theta)), self.lfd)

#         msg = xycar_motor()
#         msg.angle = Angle
#         msg.speed = Speed
#         self.pub.publish(msg)




