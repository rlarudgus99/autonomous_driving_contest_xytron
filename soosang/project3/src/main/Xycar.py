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
from Controller.filter import MovingAverage
class Xycar(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        # filter
        # self.angle = 0

        self.timer = Timer()
        self.sensor = XycarSensor()
        yaw0 = self.sensor.init(self.rate)

        self.lane_detector = LaneDetector()

        self.mode_controller = ModeController(self.timer)
        self.stanley_controller = StanleyController(self.timer)
        # filter
        self.maangle =MovingAverage(1)
        # speed filter000000000000000000000
        self.maspeed =MovingAverage(1)
        self.target_lane = 'middle'
        self.control_dict = {
            'straight' : self.stanley,
            'curve': self.stanley,
            'vstraight' : self.stanley,
            'vcurve': self.stanley,
            'straight1' : self.stanley,
            'straight2' : self.stanley,
            'straight3' : self.stanley,
            'straight4' : self.stanley,
            'straight5' : self.stanley,
            'curve0': self.stanley,
            'curve00': self.stanley,
            'curve1': self.stanley,
            'curve2': self.stanley,
            'curve3': self.stanley,
            'curve4': self.stanley,
            'curve5': self.stanley,
        }

    def stanley(self):
        angle, target = self.lane_detector(self.sensor.cam, self.target_lane)
        self.msg.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode(target,angle))
        print("angle: ",self.msg.angle)
        # filter
        #self.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode(),self.msg.speed)
       # print(target-340)
        self.maangle.add_sample(self.msg.angle)
        self.msg.angle = self.maangle.get_wmm()
        #if abs(-0.2*(target-320)) > 20:
        #   self.msg.speed = 2
        self.maspeed.add_sample(self.msg.speed)
        self.msg.speed = self.maspeed.get_wmm()
        print("speed: ",self.msg.speed)
        #print(self.msg.angle)
        self.msg.angle=self.msg.angle+10
        self.pub.publish(self.msg)
        self.rate.sleep()

    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        cv2.imshow('cam', self.sensor.cam)
        mode = self.mode_controller()
        self.control_dict[mode]()
        cv2.waitKey(1)
