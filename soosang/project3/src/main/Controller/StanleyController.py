#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
from Controller.filter import MovingAverage

class StanleyController(object):
    '''
    Stanley Contoller for following lane 
    '''
    def __init__(self, timer):
        self.speed = 5
        self.mavx = MovingAverage(15)
        self.timer = timer
        # speed 000000000000000000000
        self.target_speed = {
            'vstraight' : 50,
            'straight' : 45,
            'straight1' : 40,
            'straight2' : 35,
            'straight3' : 30,
            'straight4' : 25,
            'straight5' : 20,
            'curve0': 20,
            'curve00': 20,
            'curve': 20,
            'curve1': 20,
            'curve2': 18,
            'curve3': 16,
            'curve4': 14,
            'curve5': 20,
            'vcurve': 12

#            'straight' : 30,
#            'curve': 15,
#            'vstraight' : 35,
#            'vcurve': 10

        }
        self.acc = {
            'straight1' : 1.0,
            'curve1': None
        }
        self.delay = {
            'straight1' : 0.5,
            'curve1': -1
        }

    def __call__(self, angle, target, mode):
        '''
        returns angle and speed for following target
        '''
        self.speed = self.target_speed[mode]


        vx = 5.0 if (mode == 'vstraight' or mode == 'straight1' or mode == 'straight2' or mode == 'straight3' or mode == 'straight4' or mode == 'straight5' or mode == 'straight') else 1.5
        #self.mavx.add_sample(vx)
        #vx = self.mavx.get_wmm()
        yaw_term = angle
        cte = (target-320) * 1.9/650.0


        if (mode == 'vstraight'):
            stanley_k = 0.3
        elif (mode == 'straight'):
            stanley_k = 0.33
        elif (mode == 'straight5'):
            stanley_k = 0.35
        elif (mode == 'straight4'):
            stanley_k = 0.38
        elif (mode == 'straight3'):
            stanley_k = 0.40
        elif (mode == 'straight2'):
            stanley_k = 0.45
        elif (mode == 'straight1'):
            stanley_k = 0.5

        elif (mode == 'vcurve'):
            stanley_k = 1.5
        elif (mode == 'curve5'):
            stanley_k = 1.47
        elif (mode == 'curve4'):
            stanley_k = 1.44
        elif (mode == 'curve3'):
            stanley_k = 1.41
        elif (mode == 'curve2'):
            stanley_k = 1.38
        elif (mode == 'curve2'):
            stanley_k = 1.35
        elif (mode == 'curve1'):
            stanley_k = 1.32
        elif (mode == 'curve'):
            stanley_k = 1.29
        elif (mode == 'curve0'):
            stanley_k = 1.26
        elif (mode == 'curve00'):
            stanley_k = 1.23
        #stanley_k = 1.5 if (mode[:5] == 'curve'or mode == 'vcurve') else 0.5
        cte_term = np.arctan2(stanley_k*cte, vx)#22
        
        self.angle = -(np.degrees(0.4 * yaw_term + cte_term)*5.0/3.0)

        return self.angle, self.speed

#straight
#90curve
#45curve
#ucurve





class StanleyController_org(object):
    '''
    Stanley Contoller for following lane 
    '''
    def __init__(self, timer):
        self.speed = 5
        self.mavx = MovingAverage(15)
        self.timer = timer
        # speed 000000000000000000000
        self.target_speed = {
            'vstraight' : 50,
            'straight1' : 40,
            'straight2' : 30,
            'curve1': 20,
            'curve2': 17,
            'curve3': 15,
            'curve4': 12,
            'curve5': 10,
            'curve6': 9,
            'vcurve': 7

#            'straight' : 30,
#            'curve': 15,
#            'vstraight' : 35,
#            'vcurve': 10

        }
        self.acc = {
            'straight' : 1.0,
            'curve': None
        }
        self.delay = {
            'straight' : 0.5,
            'curve': -1
        }

    def __call__(self, angle, target, mode):
        '''
        returns angle and speed for following target
        '''
        self.speed = self.target_speed[mode]
        vx = 2.0 if (mode == 'curve' or mode == 'vcurve') else 4.0
        #self.mavx.add_sample(vx)
        #vx = self.mavx.get_wmm()
        yaw_term = angle
        cte = (target-320) * 1.9/650.0

        stanley_k = 1.5 if (mode == 'curve'or mode == 'vcurve') else 0.5

        cte_term = np.arctan2(stanley_k*cte, vx)#22
        
        self.angle = -(np.degrees(0.4 * yaw_term + cte_term)*5.0/3.0)

        return self.angle, self.speed













class StanleyControllerS(object):
    '''
    Stanley Contoller for following lane 
    '''
    def __init__(self, timer):
        self.speed = 5
        self.timer = timer
        self.target_speed = {
            'long straight1' : 5,
            'short straight': 45,
            'long straight2' : 50,
            'curve': 50
        }
        self.acc = {
            'long straight1' : 1.0,
            'short straight': -0.4,
            'long straight2' : 1.0,
            'curve': None
        }
        self.delay = {
            'long straight1' : 1.0,
            'short straight': 2.5,
            'long straight2' : 1.0,
            'curve': -1
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
        vx = self.speed/5
        yaw_term = angle
        cte = (target-320) * 1.9/650.0
        stanley_k = 1.0 if (mode == 'curve') else 0.7
        cte_term = np.arctan2(stanley_k*cte, 2.0)#22
        return -(np.degrees(0.4 * yaw_term + cte_term)*5.0/3.0), self.speed

