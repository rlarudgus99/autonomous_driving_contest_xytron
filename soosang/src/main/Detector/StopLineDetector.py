#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from BEV import BEV
import numpy as np

# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

class StopLineDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''
    def __init__(self):

        # BEV
        self.bev = BEV()

        # stopline detection param
        self.stopline_threshold = 150
        self.area_threshold = 2000
        self.lengh_threshold = 300
   
    def __call__(self, img):
        '''
        return True if stopline is detected else False
        '''
        bev = self.bev(img)
        blur = cv2.GaussianBlur(bev, (5, 5), 0)
        ##gray = cv2.cvtColor(bev,cv2.COLOR_BGR2GRAY)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        _, lane = cv2.threshold(L, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        ##_, lane = cv2.threshold(gray, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        detected = False
        #x1, x2, y1, y2 = 140, 500, 100, 120
        x1, x2, y1, y2 = 325-75, 325+75, 20,40
        roi = lane[y1:y2,x1:x2]
        print(np.sum(roi)/255)
        #if np.mean(roi) > 200:
        if np.sum(roi) > 1650*255:
            cv2.rectangle(lane, (x1, y1), (x2, y2), green, 2)
            detected = True

        cv2.imshow('stopline', lane)
        return detected






























##################################################################################
##################################################################################
class StopLineDetectorS(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''
    def __init__(self):

        # BEV
        self.bev = BEV()

        # stopline detection param
        self.stopline_threshold = 125
        self.area_threshold = 2000
        self.lengh_threshold = 300
   
    def __call__(self, img):
        '''
        return True if stopline is detected else False
        '''
        bev = self.bev(img)
        blur = cv2.GaussianBlur(bev, (5, 5), 0)
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        _, lane = cv2.threshold(L, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        

        detected = False
        # print('stanley')
        for cont in contours:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)

            if not ((area > self.area_threshold) and (length > self.lengh_threshold)):
                continue
            if len(cv2.approxPolyDP(cont, length*0.02, True)) != 4:
                continue

            x, y, w, h = cv2.boundingRect(cont)
            center = (x + int(w/2), y + int(h/2))
            _, width, _ = bev.shape

            if (200 <= center[0] <= (width - 200)) and (w > 400) & (h < 80):
                cv2.rectangle(bev, (x, y), (x + w, y + h), green, 2)
                detected = True

        cv2.imshow('stopline', bev)
        return detected
