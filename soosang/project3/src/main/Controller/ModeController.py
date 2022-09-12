#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ModeController(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    def __init__(self, timer):
        self.mode = 'long straight1'
        self.timer = timer
        self.lap = 0
        self.lap_target = 3
        self.target = 320
        self.angle = 0
    def set_mode(self, mode):
        self.mode = mode


    def get_mode(self,target,angle):
        self.target = target
        self.angle = angle/np.pi*180
        return self.mode

    def __call__(self):
        '''
        updates and returns current mode
        '''
        #print('a',self.angle)
#        if abs(self.target-320)<=3:
#            print('vstraight')
#            self.mode = 'vstraight'
#            self.timer.update()
#        elif abs(self.target-320)<=20:
#            print('straight')
#            self.mode = 'straight'
#            self.timer.update()
#        elif abs(self.target-320)<=60:
#            print('curve')
#            self.mode = 'curve'
#            self.timer.update()
#        else:
#            print('vcurve')
#            self.mode = 'vcurve'
#            self.timer.update()
        if abs(self.angle) <= 3:
            print('angle<0.1:',self.angle)
            if abs(self.target-320)<=3:
                print('vstraight')
                self.mode = 'vstraight'
                self.timer.update()
            elif abs(self.target-320)<=5:
                print('straight')
                self.mode = 'straight'
            elif abs(self.target-320)<=10:
                print('straight1')
                self.mode = 'straight1'
                self.timer.update()
            elif abs(self.target-320)<=15:
                print('straight2')
                self.mode = 'straight2'
                self.timer.update()
            elif abs(self.target-320)<=20:
                print('straight3')
                self.mode = 'straight3'
                self.timer.update()
            elif abs(self.target-320)<=25:
                print('straight4')
                self.mode = 'straight4'
                self.timer.update()
            elif abs(self.target-320)<=30:
                print('straight5')
                self.mode = 'straight5'
                self.timer.update()


        else:
            if abs(self.target-320)<=25:
                print('curve0')
                self.mode = 'curve0'
                self.timer.update()
            elif abs(self.target-320)<=30:
                print('curve00')
                self.mode = 'curve00'
                self.timer.update()
            elif abs(self.target-320)<=35:
                print('curve')
                self.mode = 'curve'
                self.timer.update()
            elif abs(self.target-320)<=40:
                print('curve1')
                self.mode = 'curve1'
                self.timer.update()
            elif abs(self.target-320)<=50:
                print('curve2')
                self.mode = 'curve2'
                self.timer.update()
            elif abs(self.target-320)<=60:
                print('curve3')
                self.mode = 'curve3'
                self.timer.update()
            elif abs(self.target-320)<=70:
                print('curve4')
                self.mode = 'curve4'
                self.timer.update()
            elif abs(self.target-320)<=80:
                print('curve5')
                self.mode = 'curve5'
                self.timer.update()
            else:
                print('vcurve')
                self.mode = 'vcurve'
                self.timer.update()

        return self.mode

class ModeControllerO(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    def __init__(self, yaw0, timer):
        self.mode = 'straight'
        self.timer = timer
        self.yaw0 = yaw0
        self.lap = 0
        self.lap_target = 3

    def set_mode(self, mode):
        self.mode = mode

    def set_yaw0(self, yaw0):
        self.yaw0 = yaw0

    def get_mode(self):
        return self.mode

    def __call__(self, yaw):
        '''
        updates and returns current mode
        '''
        print("yaw: ",yaw)
        diff_yaw = abs(yaw - self.yaw0)
        if diff_yaw > np.pi:
            diff_yaw = 2*np.pi - diff_yaw
        if self.mode == 'long straight1' and  np.pi/2.0 - 0.1 < diff_yaw < np.pi/2.0 + 0.1:
            print('short straight')
            self.mode = 'short straight'
            self.timer.update()
        elif self.mode == 'short straight' and  np.pi - 0.15 < diff_yaw < np.pi + 0.15:
            print('long straight2')
            self.timer.update()
            self.mode = 'long straight2'
        elif self.mode == 'curve' and  diff_yaw < 0.05:
            self.lap += 1
            print('finish lap {}'.format(self.lap))
            self.timer.update()
            if self.lap < self.lap_target:
                self.mode = 'long straight1'
                #self.stanley_k = 0.7
        return self.mode
