#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from collections import deque
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

class XycarVision(object):

    def __init__(self):
        
        # read usb_cam
        self.img = None
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)

        # canny params
        self.canny_low, self.canny_high = 100, 120

        # calibration config
        self.img_size = (640, 480)
        self.warp_img_w, self.warp_img_h, self.warp_img_mid = 650, 120, 60

        self.mtx = np.array([[363.090103, 0.000000, 313.080058],
                             [0.000000, 364.868860, 252.739984],
                             [0.000000, 0.000000, 1.000000]])
        self.dist = np.array([-0.334146, 0.099765, -0.000050, 0.001451, 0.000000])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, self.img_size, 1, self.img_size)

        # perspective config
        warpx_mid, warpx_margin_hi, warpx_margin_lo, warpy_hi, warpy_lo, tilt = 320, 200, 319, 325, 375, -5
        self.warp_src  = np.array([[warpx_mid+tilt-warpx_margin_hi, warpy_hi], [warpx_mid+tilt+warpx_margin_hi, warpy_hi], 
                                   [warpx_mid-warpx_margin_lo,  warpy_lo], [warpx_mid+warpx_margin_lo, warpy_lo]], dtype=np.float32)
        self.warp_dist = np.array([[100, 0], [649-100, 0],
                                   [100, 119], [649-100, 119]], dtype=np.float32)
        self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dist)

        # HoughLineP params
        self.hough_threshold, self.min_length, self.min_gap = 10, 50, 10

        # filtering params:
        self.angle_tolerance = np.radians(30)
        self.cluster_threshold = 25

        # initial state
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=8)
        self.lane = np.array([90.0, 320., 568.])

    def callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def to_binary(self, img, show=False):
        img = cv2.GaussianBlur(img, (7,7), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if show:
            cv2.imshow('binary', img)
        return img

    def to_calibrated(self, img):
        tf_image = cv2.undistort(img, self.mtx, self.dist, None, self.cal_mtx)
        return tf_image

    def to_perspective(self, img):
        return cv2.warpPerspective(img, self.M, (self.warp_img_w, self.warp_img_h), flags=cv2.INTER_LINEAR)

    def to_bev(self, img, show=False):
        img = self.to_calibrated(img)
        img = self.to_perspective(img)
        if show:
            cv2.imshow('bev', img)
        return img

    def hough(self, img, show=False):
        lines = cv2.HoughLinesP(img, 1, np.pi/180, self.hough_threshold, self.min_gap, self.min_length)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), red, 2)
            cv2.imshow('hough', hough_img)
        return lines

    def filter(self, lines, show=True):
        thetas, positions = [], []
        if show:
            filter_img = np.zeros((self.warp_img_h, self.warp_img_w, 3))

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if y1 == y2:
                    continue
                flag = 1 if y1-y2 > 0 else -1
                theta = np.arctan2(flag * (x2-x1), flag * 0.9* (y1-y2))
                if abs(theta - self.angle) < self.angle_tolerance:
                    position = float((x2-x1)*(self.warp_img_mid-y1))/(y2-y1) + x1
                    thetas.append(theta)
                    positions.append(position) 
                    if show:
                        cv2.line(filter_img, (x1, y1), (x2, y2), red, 2)

        self.prev_angle.append(self.angle)
        if thetas:
            self.angle = np.mean(thetas)
        if show:
            cv2.imshow('filtered lines', filter_img)
        return positions

    def get_cluster(self, positions):
        clusters = []
        for position in positions:
            if 0 <= position < self.warp_img_w:
                for cluster in clusters:
                    if abs(cluster[0] - position) < self.cluster_threshold:
                        cluster.append(position)
                        break
                else:
                    clusters.append([position])
        lane_candidates = [np.mean(cluster) for cluster in clusters]
        return lane_candidates

    def predict_lane(self):
        predicted_lane = self.lane[1] + [-220/max(np.cos(self.angle), 0.75), 0, 240/max(np.cos(self.angle), 0.75)]
        predicted_lane = predicted_lane + (self.angle - np.mean(self.prev_angle))*70
        return predicted_lane

    def update_lane(self, lane_candidates, predicted_lane):

        if not lane_candidates:
            self.lane = predicted_lane
            return

        possibles = []

        for lc in lane_candidates:

            idx = np.argmin(abs(self.lane-lc))

            if idx == 0:
                estimated_lane = [lc, lc + 220/max(np.cos(self.angle), 0.75), lc + 460/max(np.cos(self.angle), 0.75)]
                lc2_candidate, lc3_candidate = [], []
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < 50 :
                        lc2_candidate.append(lc2)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < 50 :
                        lc3_candidate.append(lc3)
                if not lc2_candidate:
                    lc2_candidate.append(lc + 220/max(np.cos(self.angle), 0.75))
                if not lc3_candidate:
                    lc3_candidate.append(lc + 460/max(np.cos(self.angle), 0.75))
                for lc2 in lc2_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc, lc2, lc3])

            elif idx == 1:
                estimated_lane = [lc - 220/max(np.cos(self.angle), 0.75), lc, lc + 240/max(np.cos(self.angle), 0.75)]
                lc1_candidate, lc3_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < 50 :
                        lc1_candidate.append(lc1)
                for lc3 in lane_candidates:
                    if abs(lc3-estimated_lane[2]) < 50 :
                        lc3_candidate.append(lc3)
                if not lc1_candidate:
                    lc1_candidate.append(estimated_lane[0])
                if not lc3_candidate:
                    lc3_candidate.append(estimated_lane[2])
                for lc1 in lc1_candidate:
                    for lc3 in lc3_candidate:
                        possibles.append([lc1, lc, lc3])

            else :
                estimated_lane = [lc - 460/max(np.cos(self.angle), 0.75), lc - 240/max(np.cos(self.angle), 0.75), lc]
                lc1_candidate, lc2_candidate = [], []
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < 50 :
                        lc1_candidate.append(lc1)
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < 50 :
                        lc2_candidate.append(lc2)
                if not lc1_candidate:
                    lc1_candidate.append(estimated_lane[0])
                if not lc2_candidate:
                    lc2_candidate.append(estimated_lane[1])
                for lc1 in lc1_candidate:
                    for lc2 in lc2_candidate:
                        possibles.append([lc1, lc2, lc])

        possibles = np.array(possibles)
        error = np.sum((possibles-predicted_lane)**2, axis=1)
        best = possibles[np.argmin(error)]
        self.lane = 0.4 * best + 0.6 * predicted_lane

    def mark_lane(self, img, lane=None):
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lane is None:
            lane = self.lane
        l1, l2, l3 = self.lane
        cv2.circle(img, (int(l1), self.warp_img_mid), 3, yellow, 5, cv2.FILLED)
        cv2.circle(img, (int(l2), self.warp_img_mid), 3, green, 5, cv2.FILLED)
        cv2.circle(img, (int(l3), self.warp_img_mid), 3, blue, 5, cv2.FILLED)
        cv2.imshow('marked', img)

    def update(self):
        cv2.imshow('cam', self.img)
        binary = self.to_binary(self.img, show=False)
        bev = self.to_bev(binary, show=False)
        lines = self.hough(bev, show=False)
        positions = self.filter(lines, show=False)
        lane_candidates = self.get_cluster(positions)
        predicted_lane = self.predict_lane()
        self.update_lane(lane_candidates, predicted_lane)
        self.mark_lane(bev)
        return self.angle, self.lane[1]
