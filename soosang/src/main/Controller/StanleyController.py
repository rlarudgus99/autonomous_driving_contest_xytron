#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class StanleyController(object):
    '''
    Stanley Contoller for following lane 
    '''
    def __init__(self, timer):
        self.speed = 20
        self.timer = timer
        self.target_speed = {
            'long straight' : 20,
            'short straight': 45,
            'obstacle': 35,
            'stopline': 5,
            'curve': 20,
            'parking_H': 20,
            'parking_T': 20
        }
        self.acc = {
            'long straight' : 1.0,
            'short straight': -0.4,
            'obstacle': None,
            'stopline': -2.0,
            'curve': None,
            'parking_H': -0.6,
            'parking_T': -0.6
        }
        self.delay = {
            'long straight' : 1.0,
            'short straight': 2.5,
            'obstacle': -1,
            'stopline': 2.8,
            'curve': -1,
            'parking_H': 1.0,
            'parking_T': 1.0
        }

    def __call__(self, angle, target, mode):
        '''
        returns angle and speed for following target
        '''

        if self.timer() > self.delay[mode]:
            if self.acc[mode] is None:
                self.speed = self.target_speed[mode]
            elif self.acc[mode] > 0:
                self.speed = min(self.speed+self.acc[mode], self.target_speed[mode])
            else:
                self.speed = max(self.speed+self.acc[mode], self.target_speed[mode])

        yaw_term = angle
        cte = (target-320) * 1.9/650.0
        if (mode ==  'short straight' or mode =='parking_T'):
            stanley_k = 1.0
            cte_term = np.arctan2(stanley_k*cte, 3.0)
        elif (mode == 'obstacle' or mode == 'curve'):
            stanley_k = 1.5
            cte_term = np.arctan2(stanley_k*cte, 3.0)
        else :
            cte = (target-230) * 1.9/650.0
            stanley_k = 0.7
            cte_term = np.arctan2(stanley_k*cte, 3.0)

        if (mode == 'obstacle' or mode ==  'short straight'):
            return -(np.degrees(0.4 * yaw_term + cte_term) * 5.0 / 3.0-10), self.speed
        elif mode == 'curve':
            return -(np.degrees(0.4 * yaw_term + cte_term) * 5.0 / 3.0-10), self.speed
        else:
            return -(np.degrees(0.4 * yaw_term + cte_term) * 5.0 / 3.0-10), self.speed

