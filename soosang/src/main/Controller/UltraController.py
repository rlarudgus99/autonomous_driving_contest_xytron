#! /usr/bin/env python
# -*- coding:utf-8 -*-

class UltraController(object):
    '''
    Speed and Steer controller for precise parking using Ultrasonic Data
    '''
    def __init__(self):
        self.reverse = True

    def __call__(self, right_back, left_back, center_back, right, left):
        '''
        return angle and speed from ultrasonic
        '''
        # direction
        if self.reverse:
            if center_back < 7:
                self.reverse = False
        else:
            if center_back > 20:
                self.reverse = True

        # termination
        if 1: #and (abs(right - left) < 10):
            print('we will be soosang, you know?')
            return 0, 0

        if abs(right - left) < 10:
            angle = 6.5
        elif right > left :
            angle = -23.5
        elif right < left :
            angle = 36.5

        if self.reverse :
            speed = -3
        else :
            speed = 3
        
        return angle, speed
