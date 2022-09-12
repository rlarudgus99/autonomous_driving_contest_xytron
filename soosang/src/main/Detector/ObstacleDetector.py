#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ObstacleDetector(object):
    '''
    Detects obstacle using lidar data
    '''

    def __init__(self, timer):

        self.avoid_direction = 'middle'
        self.obstacle_counter = 0
        self.timer = timer
        self.obs_dict = {1:1.3, 2:3.4, 3:5.4, 4:6.8, 5:7.7}

    def __call__(self, ranges, angle_increment):
        '''
        returns direction to avoid obstacles from lidar inputs
        '''

        if self.obstacle_counter == 0 and (self.timer() > 1.0):
            ranges = np.array(ranges)

            ranges[180:] = 0.0
            deg = np.arange(360) * angle_increment - 180  * angle_increment

            #print(len(deg))
            mask = (np.abs(ranges * np.sin(deg)) > 0.2 ) & (np.abs(ranges * np.sin(deg)) < 0.5) & (-0.4 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 0.4)
            filtered = np.where(mask, ranges, 0.0)
            # print(filtered)
            nz = np.nonzero(filtered)[0]
            print(nz)
            if len(nz) > 5:
                if np.median(nz) > 90:
                    self.avoid_direction = 'left'
                else:
                    self.avoid_direction = 'right'
                print('avoid to ' + self.avoid_direction)
                self.timer.update()
                self.obstacle_counter += 1

        elif self.obstacle_counter == 5:
            if self.timer() > self.obs_dict[self.obstacle_counter]:
               self.avoid_direction = 'middle'
               print('avoid to ' + self.avoid_direction + 'ctr == five')
               self.obstacle_counter += 1

        elif self.obstacle_counter != 0:
            if self.timer() > self.obs_dict[self.obstacle_counter]:
               self.avoid_direction = 'left' if self.avoid_direction == 'right' else 'right'
               print('avoid to ' + self.avoid_direction)
               self.obstacle_counter += 1



        return self.avoid_direction
