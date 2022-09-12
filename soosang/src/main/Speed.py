#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from Timer import Timer
from XycarSensor import XycarSensor
from Detector.LaneDetector import LaneDetector
from Controller.ModeController import ModeController
from Controller.StanleyController import StanleyController

class Xycar(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.timer = Timer()
        self.sensor = XycarSensor()
        yaw0 = self.sensor.init(self.rate)

        
        self.lane_detector = LaneDetector()

        self.mode_controller = ModeController(yaw0, self.timer)
        self.stanley_controller = StanleyController(self.timer)


        self.target_lane = 'middle'
        self.control_dict = {
            'long straight' : self.stanley,
            'short straight': self.stanley,
            'curve': self.stanley,
        }

    def stanley(self):
        angle, target = self.lane_detector(self.sensor.cam, self.target_lane)
        self.msg.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode())
        self.pub.publish(self.msg)
        self.rate.sleep()

    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        cv2.imshow('cam', self.sensor.cam)
        mode = self.mode_controller(self.sensor.yaw)
        self.control_dict[mode]()
        cv2.waitKey(1)
