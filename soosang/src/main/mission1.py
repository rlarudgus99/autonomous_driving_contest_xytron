#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from Speed import Xycar

rospy.init_node('test')

xycar = Xycar()

while not rospy.is_shutdown():
    xycar.control()
