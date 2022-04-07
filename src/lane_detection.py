#!/usr/bin/env python
# # -*- coding: utf-8 -*-

import cv2
import numpy as np
from cv2 import TickMeter as tm
import glob, os


class undistort:
    def __init__(self):
        self.frame_cnt = 1
        self.calib_path = "./calib.npz"

    def capture(self, frame):
        wk = cv2.waitKey(1)
        if wk == ord("c"):
            file_name = "orig_{0}.jpg".format(self.frame_cnt)
            cv2.imwrite(filename=file_name, img=frame)
            self.frame_cnt += 1

        elif wk == ord("q"):
            self.calibrate()

        if self.frame_cnt > 15:
            self.calibrate()

    def calibrate(self, nx=8, ny=6):
        objp = np.zeros((nx * ny, 3), np.float32)
        objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

        objpoints = []  # 3d points in real world space
        imgpoints = []  # 2d points in image plane.

        images = glob.glob("orig_*.jpg")
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
            cv2.imshow("input image", img)
            cv2.waitKey(500)

        cv2.destroyAllWindows()

        # calibrate the camera
        img_size = (img.shape[1], img.shape[0])
        ret, mtx, dist, _, _ = cv2.calibrateCamera(
            objpoints, imgpoints, img_size, None, None
        )

        np.savez("./calib.npz", mtx=mtx, dist=dist)


class lanedetection:
    def __init__(self, mode="gaussian_otsu"):
        self.calib_path = "./calib.npz"
        self.load_calib()
        self.mode = mode
        self.mode_dict = {
            "gaussian_canny": self.gaussian_canny,
            "gaussian_otsu": self.gaussian_otsu,
            "mean_adaptive_dilate": self.mean_adaptive_dilate,
            "gaussian_adaptive_dilate": self.gaussian_adaptive_dilate,
            "test": self.bin_test,
        }

        self.calc_bev_mat()

    def calc_bev_mat(self):
        # orig img params
        w1, h1 = 640, 480
        h1_offset = 60
        pts1 = np.float32(
            [
                [0, h1 / 2 + h1_offset],
                [w1 - 1, h1 / 2 + h1_offset],
                [w1 - 1, h1 - 1],
                [0, h1 - 1],
            ]
        )

        # bev img params
        w2, h2 = 500, 250
        w2_offset = w2 / 8
        pts2 = np.float32(
            [
                [0, 0],
                [w2 - 1, 0],
                [w2 / 2 + w2_offset - 1, h2 - 1],
                [w2 / 2 - w2_offset - 1, h2 - 1],
            ]
        )

        # calc bev_mtx
        self.bev_mtx = cv2.getPerspectiveTransform(pts1, pts2)


    def load_calib(self):
        if os.path.isfile(self.calib_path):
            calib_data = np.load(self.calib_path)
            self.mtx = calib_data["mtx"]
            self.dist = calib_data["dist"]

        else:
            self.mtx = np.array(
                [
                    [422.037858, 0.0, 245.895397],
                    [0.0, 435.589734, 163.625535],
                    [0.0, 0.0, 1.0],
                ]
            )
            self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])

    def gaussian_canny(self, img):

        # blur
        blur = cv2.GaussianBlur(img, (5, 5), 0)

        # canny edge
        low_threshold = 60
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(blur), low_threshold, high_threshold)

        # show result
        cv2.imshow("img", img)
        cv2.imshow("edge_img", edge_img)

        return edge_img

    def gaussian_otsu(self, img):
        # gaussian blur
        blur = cv2.GaussianBlur(img, (5, 5), 0)


        # otsu threshold
        _, th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # show result
        cv2.imshow("img", img)
        cv2.imshow("otsu", th)

        return th

    def mean_adaptive_dilate(self, img):
        # gaussian blur
        blur = cv2.GaussianBlur(img, (5, 5), 0)

        # adaptive threshold (mean)
        th_mean = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 2
        )

        # dilation
        kernel = np.ones((3, 3), np.uint8)
        dilation = cv2.dilate(th_mean, kernel, iterations=2)
        # show result

        cv2.imshow("img", img)
        cv2.imshow("mean", th_mean)
        cv2.imshow("dilation", dilation)

        return th_mean

    def gaussian_adaptive_dilate(self, img):
        # gaussian blur
        img = cv2.GaussianBlur(img, (5, 5), 0)

        # adaptive threshold (gaussian)
        th_gaussian = cv2.adaptiveThreshold(
            img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 2
        )

        # dilation
        kernel = np.ones((3, 3), np.uint8)
        dilation = cv2.dilate(th_gaussian, kernel, iterations=1)

        # show result
        cv2.imshow("img", img)
        cv2.imshow("mean", th_gaussian)
        cv2.imshow("dilation", dilation)

        return th_gaussian

    def bin_test(self, img):
        # gaussian blur
        blur = cv2.GaussianBlur(img, (5, 5), 0)

        # threshold
        th_mean = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 2
        )
        th_gaussian = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 2
        )
        _, th_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # show results
        cv2.imshow("img", img)
        cv2.imshow("mean", th_mean)
        cv2.imshow("gaussian", th_gaussian)
        cv2.imshow("otsu", th_otsu)

        return None

    def bev(self, th):
        # remove lidar sensor
        cv2.circle(th, (320, 480), 110, 0, -1)

        # return warp
        return cv2.warpPerspective(th, self.bev_mtx, (500, 250))

    def __call__(self, image):
        if image is not None:
            # cvt to gray
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # undistort image
            gray = cv2.undistort(gray, self.mtx, self.dist, None, self.mtx)

            # add/subtrack gray intensity
            offset = -100
            gray = cv2.add(gray, offset)

            # calc binary image
            bin_gray = self.mode_dict[self.mode](gray)

            # create bev binary image
            bin_bev = self.bev(bin_gray)

            # check result
            cv2.imshow("bev", bin_bev)
            cv2.waitKey(1)
            return bin_bev
        return None


if __name__ == "__main__":
    ld = lanedetection(mode="gaussian_otsu")
    img = cv2.imread("/home/killywonka/xycar_ws/src/xycar_lkas/src/test.jpg")
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    th = ld.gaussian_otsu(gray)

    bev = ld.bev(th)
    cv2.imshow("bev", bev)
    cv2.waitKey(-1)

