#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_msgs.msg import xycar_motor
from Timer import Timer
from XycarSensor import XycarSensor
from Detector.LaneDetector import LaneDetector
from Detector.StopLineDetector import StopLineDetector
from Detector.ObstacleDetector import ObstacleDetector
from Controller.ModeController import ModeController
from Controller.ARController import ARController
from Controller.StanleyController import StanleyController
from Controller.TunnelController import TunnelController
from Controller.UltraController import UltraController
from Controller.filter import MovingAverage

class Xycar(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()
        self.maangle =MovingAverage(5)
        self.timer = Timer()
        self.sensor = XycarSensor()
        yaw0 = self.sensor.init(self.rate)

        self.obstacle_detector = ObstacleDetector(self.timer)
        self.lane_detector = LaneDetector()
        self.stopline_detector = StopLineDetector()

        self.mode_controller = ModeController(yaw0, self.timer,self.sensor)
        self.stanley_controller = StanleyController(self.timer)
        self.ar_controller = ARController()
        self.ultra_controller = UltraController()
        self.tunnel_controller = TunnelController()

        self.target_lane = 'middle'
        self.control_dict = {
            'long straight' : self.stanley,
            'short straight': self.stanley,
            'obstacle': self.obstacle,
            'stopline': self.stopline,
            'curve': self.stanley,
            'arparking': self.arparking,
            'poweroff' : self.poweroff,
            'tunnel' : self.tunnel_drive,
	   		'parking_T' : self.parking_T,
	    	'parking_H' : self.parking_H,
            'parking_stanley' : self.parking_stanley,
	    	'ultraparking' : self.ultraparking,
	    	'stop' : self.stop,
            'stop6s' : self.stop6s,
	    	'escape' : self.escape
			}

    def obstacle(self):
        #print('find obstacles')
        self.target_lane = self.obstacle_detector(self.sensor.lidar, self.sensor.angle_increment)
        if self.obstacle_detector.obstacle_counter == 6:
            print('detecting stopline...')
            self.obstacle_detector.obstacle_counter = 0
            self.mode_controller.set_mode("stopline")
        #print("stanley2")
        self.stanley()

    def stopline(self):        
        if self.stopline_detector(self.sensor.cam) :
            print('stop6s')
            self.stop6s()
        else:
            #print('stanley')
            self.stanley()


    def stop6s(self):
        print("stop for 5s...")
        yaws = []
        for _ in xrange(5):
            yaws.append(self.sensor.yaw)
            self.msg.angle, self.msg.speed = -1, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in xrange(50):
            yaws.append(self.sensor.yaw)
            self.msg.angle, self.msg.speed = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        print('go!')
        yaw0 = (np.mean(yaws) - np.pi) % (2*np.pi)
        self.mode_controller.set_yaw0(yaw0)
        self.mode_controller.set_mode('curve')


    def parking_stanley(self):       
        angle, target = self.lane_detector(self.sensor.cam, self.target_lane)
        self.msg.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode())
        self.pub.publish(self.msg)
        self.rate.sleep()

    def parking_H(self):
        
        for _ in xrange(27):
            print('parking_stanley')
            self.parking_stanley()
            self.rate.sleep()
        print('parking_H')
        for _ in xrange(23):
            self.msg.angle, self.msg.speed = -35, -10
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in xrange(15):
            self.msg.angle, self.msg.speed = +50, -10
            self.pub.publish(self.msg)
            self.rate.sleep()
       # for _ in xrange(1):
       #     self.msg.angle, self.msg.speed = 0, +10
       #     self.pub.publish(self.msg)
       #     self.rate.sleep()
        self.msg.angle, self.msg.speed = 0, -1
        self.pub.publish(self.msg)

        self.mode_controller.set_mode('arparking')


    def parking_T(self):
        print('parking_T')
# lane detection and timing
# go straight 1.2m
      #  for _ in xrange(13):
      #      self.msg.angle, self.msg.speed = 20, 20
      #      self.pub.publish(self.msg)
      #      self.rate.sleep()

        for _ in xrange(25):
            print('parking_stanley')
            self.parking_stanley()
            self.rate.sleep()

        for _ in xrange(12):
            print('left')
            self.msg.angle, self.msg.speed = 30, 20
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in xrange(5):
            print('back_straight')

            self.msg.angle, self.msg.speed = 0, -10
            self.pub.publish(self.msg)
            self.rate.sleep()

        for _ in xrange(20):
            print('back_right')

            self.msg.angle, self.msg.speed = -40, -5
            self.pub.publish(self.msg)
            self.rate.sleep()

        for _ in xrange(5):
            print('back_straight')

            self.msg.angle, self.msg.speed = 0, -10
            self.pub.publish(self.msg)
            self.rate.sleep()

        self.mode_controller.set_mode('ultraparking')



    def arparking(self):
        print('arparking')
        self.msg.angle, self.msg.speed = self.ar_controller(self.sensor.ar_x, self.sensor.ar_y, self.sensor.ar_yaw)
        self.pub.publish(self.msg)
        if self.msg.speed == 0:
            self.mode_controller.set_mode('stop')
        self.timer.update()

        # self.rate.sleep()

    def ultraparking(self):
        print('ultraparking')
        self.msg.angle, self.msg.speed = self.ultra_controller(self.sensor.right_back, self.sensor.left_back, self.sensor.center_back,self.sensor.right, self.sensor.left)
        self.pub.publish(self.msg)
        if self.msg.speed == 0:
            self.mode_controller.set_mode('stop')
        self.timer.update()
        # self.rate.sleep()

    def stop(self):
        print("stop for 3s...")
        for _ in xrange(34):
           self.msg.angle, self.msg.speed = 0, 0
           self.pub.publish(self.msg)
           self.rate.sleep()
        self.mode_controller.set_mode('escape')

    def escape(self):
	# parking_H escape
        if self.sensor.ar_id == 2:
            print('parking_H escape')
            for _ in xrange(10):
                self.msg.angle, self.msg.speed =0, -5
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in xrange(25):
                self.msg.angle, self.msg.speed = 50, 10
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in xrange(20):
                self.msg.angle, self.msg.speed = -50, 10
                self.pub.publish(self.msg)
                self.rate.sleep()
            self.mode_controller.set_mode('long straight')		

	# parking_T escape		
        else:
            print('parking_T escape')
            for _ in xrange(9):
                self.msg.angle, self.msg.speed=0,10
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in xrange(23):
                self.msg.angle, self.msg.speed=-35,10
                self.pub.publish(self.msg)
                self.rate.sleep()
            for _ in xrange(5):
                self.msg.angle, self.msg.speed=0,10
                self.pub.publish(self.msg)
                self.rate.sleep()
            #for _ in xrange(43):
            #    self.msg.angle, self.msg.speed = 6.5, 10
            #    self.pub.publish(self.msg)
            #    self.rate.sleep()
            self.mode_controller.set_mode('short straight')

     

   # self.stanley()	# check


    def tunnel_drive(self):
        Lr = self.sensor.lidar[10]
        Rr = self.sensor.lidar[170]
        self.msg.angle, self.msg.speed = self.tunnel_controller(Lr, Rr)
        self.pub.publish(self.msg)
        self.rate.sleep()
	

    def poweroff(self):
        print('poweroff')
        self.msg.speed, self.msg.angle = 0, 0
        self.pub.publish(self.msg)
        self.rate.sleep()

    def stanley(self):
       # print('stanley')
        angle, target = self.lane_detector(self.sensor.cam, self.target_lane)
        #print(angle)
        self.msg.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode())
        #print(self.mode_controller.get_mode())
        h1,h2,w1,w2 = 100,380,140,500
        img = self.sensor.cam
        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        roi = img_gray#[h1 : h2, w1 : w2]
        sum1 = np.ndarray.sum(roi)
        mean1 = sum1/640/480#(h2-h1)/(w2-w1)
        print(mean1)
        if mean1 < 40 :
            while True:
                print('tunnel')
                self.msg.angle, self.msg.speed = self.tunnel_controller(self.sensor.lidar[5:10],self.sensor.lidar[170:175])
                #self.maangle.add_sample(self.msg.angle)
                #self.msg.angle = self.maangle.get_wmm()
	        self.pub.publish(self.msg)
                img = self.sensor.cam
                img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
                roi = img_gray[0 : 480, 320 : 640]
                sum1 = np.ndarray.sum(roi)
                mean1 = sum1/640/480*2#/(h2-h1)/(w2-w1)
                print(mean1)
                #if self.sensor.ar_id == 8:
                if mean1 > 80:
                    for _ in xrange(5):
                        self.msg.angle, self.msg.speed=50,10
                        self.pub.publish(self.msg)
                        self.rate.sleep()
                    for _ in xrange(3):
                        self.msg.angle, self.msg.speed=0,10
                        self.pub.publish(self.msg)
                        self.rate.sleep()
                    self.lane_detector = LaneDetector()
                    break
        self.pub.publish(self.msg)
        self.rate.sleep()





    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        cv2.imshow('cam', self.sensor.cam)
        mode = self.mode_controller(self.sensor.yaw)
        print(mode)
        self.control_dict[mode]()
        cv2.waitKey(1)
