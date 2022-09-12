#! /usr/bin/env python
# -*- coding:utf-8 -*-

class ARController(object):
    '''
    Speed and Steer controller for precise parking using AR tag
    '''
    def __init__(self):
        self.offset = 15

    def __call__(self, x, y, yaw):
        '''
        return angle and speed from x, y, yaw of a AR tag
        '''
        # direction


        self.offset = -x * 150+9
  
        # termination
        
        if 0.23 < y < 0.32 :
            # and abs(yaw) < 0.06
            print('parking')
            return -1, 0
        else:
            if x < 0.016:   
                angle = self.offset
            else :
                angle = -self.offset
            #angle = -self.offset +20 if y < 0.35 else self.offset + 20     
        speed = 3.0
        return 0, speed

