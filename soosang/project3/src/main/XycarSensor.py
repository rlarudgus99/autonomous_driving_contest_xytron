#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class XycarSensor(object):
    '''
    Class for receiving and recording datas from Xycar's Sensors
    '''
    def __init__(self):
        
        # camera sensor
        self.cam = None
        self.bridge = CvBridge()
        self.sub_cam = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_cam)

        # imu sensor
       # self.yaw = None
        #self.sub_imu = rospy.Subscriber('imu', Imu, self.callback_imu, queue_size=1)

    def callback_cam(self, msg):
        self.cam = self.bridge.imgmsg_to_cv2(msg, "bgr8")



    def init(self, rate):
        '''
        wait for initial callbacks from all sensors
        set initial yaw 
        '''

        # ready usb_cam
        while self.cam is None:
            rate.sleep()
        print("usb_cam ready")
        


