#! /usr/bin/env python
# -*- coding:utf-8 -*-

class TunnelController(object):
    '''
    Lidar
    '''

    #def __init__(self):

    def __call__(self, Lr, Rr):

        # direction 
        if min(Lr)>min(Rr):
            return 45, 3
        elif min(Lr) < 0.3:
            return -20,3
        else:
            return 8, 3


