#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu

warpx_margin = 20
warpy_margin = 3

warpx_margin = 20
warpy_margin = 3

nwindows = 9
margin = 12
minpix = 5
lane_bin_th = 145

Width = 640
Height = 400

warp_img_w = 320
warp_img_h = 240

warp_src  = np.array([
    [230-warpx_margin, 300-warpy_margin],  
    [45-warpx_margin,  450+warpy_margin],
    [445+warpx_margin, 300-warpy_margin],
    [610+warpx_margin, 450+warpy_margin]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)


def lane_estimation(image_binary):
    if image_binary is not None:
        image_binary = image_binary[:400,:640]
        cv2.imshow("ib", image_binary)
        #mask = np.zeros_like(image_binary)
        #cv2.rectangle(mask, (400,0), (480,640),(255,255,255),2)
        #cv2.copyTo(image_binary,mask,image_binary)
        birdview_img, M, minv = warp_image(image_binary, warp_src, warp_dist, (warp_img_w, warp_img_h))
        cte, curve = get_mid_poly(birdview_img)
        return cte,curve

def get_angle_between_lines(intersect, point1, point2):
    return math.atan2((point2[1]-intersect[1]), (point2[0]-intersect[0])) - math.atan2((point1[1]-intersect[1]), (point1[0]-intersect[0]))

def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def get_mid_poly(lane):
    global nwindows
    global margin
    global minpix
    global lane_bin_th

    histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)      
    midpoint = np.int(histogram.shape[0]/2)
    leftx_current = np.argmax(histogram[:midpoint])
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint

    # todo : left/right lane이 보이지 않을때도 고려해야함

    window_height = np.int(lane.shape[0]/nwindows)
    nz = lane.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    
    mx, my= [], []

    out_img = np.dstack((lane, lane, lane))*255

    for window in range(nwindows):

        win_yl = lane.shape[0] - (window+1)*window_height
        win_yh = lane.shape[0] - window*window_height

        win_xll = leftx_current - margin
        win_xlh = leftx_current + margin
        win_xrl = rightx_current - margin
        win_xrh = rightx_current + margin

        #cv2.rectangle(out_img, (win_xll, win_yl), (win_xlh, win_yh), (0,255,0),2)
        #v2.rectangle(out_img, (win_xrl, win_yl), (win_xrh, win_yh), (0,255,0),2)

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nz[1][good_right_inds]))
	
        mx.append((leftx_current+rightx_current)/2)
        my.append((win_yl + win_yh)/2)



    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    midindex = nwindows//2
    cte = mx[midindex]-(warp_img_w/2)
    #curve = get_angle_between_lines((mx[midindex],my[midindex]),(mx[midindex-1],my[midindex-1]),(mx[midindex+1],my[midindex+1]))

    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
    center_x, center_y, r = mx[midindex], my[midindex], 5
    cv2.circle(out_img, (int(center_x), int(center_y)), r, (255,0,0), 2) 
    cv2.imshow("viewer", out_img)

    
    return cte, 0
