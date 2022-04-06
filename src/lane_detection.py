#!/usr/bin/env python 
# # -*- coding: utf-8 -*- 

import cv2 
import numpy as np 
from cv2 import TickMeter as tm
import glob, os

class undistort:
    def __init__(self):
        self.frame_cnt=1
        self.calib_path="./calib.npz"

    def capture(self, frame):
        wk = cv2.waitKey(1)
        if wk==ord('c'):
            file_name = "orig_{0}.jpg".format(self.frame_cnt)    
            cv2.imwrite(filename=file_name, img=frame)
            self.frame_cnt+=1

        elif wk==ord('q'):
            self.calibrate()          

        if self.frame_cnt > 15:
            self.calibrate()
    
    def calibrate(self, nx=8, ny=6):
        objp = np.zeros((nx*ny,3), np.float32)
        objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)
        
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.

        images = glob.glob('orig_*.jpg')
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (nx,ny),None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (nx,ny), corners, ret)
            cv2.imshow('input image',img)
            cv2.waitKey(500)

        cv2.destroyAllWindows()

        # calibrate the camera
        img_size = (img.shape[1], img.shape[0])
        ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

        np.savez("./calib.npz", mtx=mtx, dist=dist)         

class lanedetection:
    def __init__(self):
        self.calib_path="./calib.npz"
        self.load_calib()

    def load_calib(self):
        if os.path.isfile(self.calib_path):
            calib_data=np.load(self.calib_path)
            self.mtx=calib_data["mtx"]
            self.dist=calib_data["dist"]

        else:
            self.mtx = np.array([
                    [422.037858, 0.0, 245.895397], 
                    [0.0, 435.589734, 163.625535], 
                    [0.0, 0.0, 1.0]
                    ])
            self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
        
    def gaussian_canny(self):
        # blur
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(self.gray,(kernel_size, kernel_size), 0)

        # canny edge
        low_threshold = 60
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
        cv2.imshow("edge_img", edge_img)
        cv2.waitKey(1)
        return edge_img
    
    def run(self, image):
        # undistort image
        if image is not None:
            image = cv2.undistort(image, self.mtx, self.dist, None, self.mtx)
            self.gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            return self.gaussian_canny()
        return None