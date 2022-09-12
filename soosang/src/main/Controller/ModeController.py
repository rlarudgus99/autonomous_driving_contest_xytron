#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import cv2

class ModeController(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    def __init__(self, yaw0, timer,sensor):
        self.mode = 'long straight'
        self.timer = timer
        self.yaw0 = yaw0
        # self.lap = 0
        # self.lap_target = 3
        self.sensor = sensor
        self.parkT = 0

    def set_mode(self, mode):
        self.mode = mode

    def set_yaw0(self, yaw0):
        self.yaw0 = yaw0

    def get_mode(self):
        return self.mode
    
    def roi_chk(self):
        h1,h2,w1,w2 = 100,380,140,500
        img = self.sensor.cam
        img_gray = cv2.cvtColor(img,cv2.IMREAD_BGR2GRAY)
        roi = img_gray[h1 : h2, w1 : w2]
        sum1 = np.ndarray.sum(roi)
        mean1 = sum1/(h2-h1)/(w2-w1)
        if mean1 < 50 :
            return 1
        else:
            return 0

    def __call__(self, yaw):
        '''
        updates and returns current mode
        '''
        diff_yaw = abs(yaw - self.yaw0)
        print("yaw: ",diff_yaw)
#        if diff_yaw > np.pi:
#            diff_yaw = 2*np.pi - diff_yaw

	
	# finding parking_H
        if self.sensor.ar_id == 1 and self.sensor.ar_y < 1.2:
	    self.mode = 'parking_H'

        # tunnel
        # if self.mode == 'long straight' and self.roi_chk == 1:
        #     self.mode = 'tunnel'
	# if self.mode == 'tunnel' and self.roi_chk == 0:
	#     self.mode = 'short straight'

	# finding parking_T
	if self.sensor.ar_id == 8 and self.sensor.ar_y < 1.2 and self.parkT == 0:
 # and self.mode != 'stop' and self.mode != 'ultraparking' and self.mode != 'escape':
	    self.mode = 'parking_T'
            self.parkT = 1
           



        # if self.mode == 'short straight' and  np.pi - 0.15 < diff_yaw < np.pi + 0.15:
        # if self.mode == 'short straight' :


#	if self.mode != 'stop' and self.mode != 'ultraparking' and self.mode != 'escape' and self.mode != 'obstacle' and self.mode != 'stopline' and self.mode != 'long straight' and self.mode == 'short straight' and self.sensor.ar_id == 8 :

        if self.mode == 'short straight' and 2.64 - 1 < diff_yaw < 2.64 +1 :
# and  np.pi - 0.3-0.36 < diff_yaw < np.pi + 0.3-0.36:
            print('detecting obstacle...')
            self.timer.update()
            self.mode = 'obstacle'
        return self.mode
