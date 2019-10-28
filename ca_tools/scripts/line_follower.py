#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) Ekumen
# Released under the BSD License.
#
# Authors:
#   * Ramiro Serra

import math
import signal
import sys
from math import pi
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


LINEAR_VEL = 0.05

ANGULAR_VEL = 0.2

VEL_PUB_TOPIC = "/create1/cmd_vel"
RIGHT_SENSOR_SUB_TOPIC = "/color_sensor_plugin/right_color_sensor"
LEFT_SENSOR_SUB_TOPIC = "/color_sensor_plugin/left_color_sensor"

class LineFollowerNode(object):
    """ A class that inits a node, that moves the robot to follow a line
    """

    def __init__(self):

        rospy.init_node('line_follower_node')

        #suscribers and publishers1)
        self._my_vel_pub = rospy.Publisher(VEL_PUB_TOPIC, Twist, queue_size=10)

        self._my_right_sensor_sub = rospy.Subscriber(
            RIGHT_SENSOR_SUB_TOPIC, Bool, self._right_sensor_callback)
        
        self._my_left_sensor_sub = rospy.Subscriber(
            LEFT_SENSOR_SUB_TOPIC, Bool, self._left_sensor_callback)

        self._is_left_detected = False
        self._is_right_detected = False

    def _left_sensor_callback(self,data):
        self._is_left_detected = data.data
    
    def _right_sensor_callback(self,data):
        self._is_right_detected = data.data
    
    def _move_forward(self):
        aux = Twist()
        aux.linear.x = LINEAR_VEL
        self._my_vel_pub.publish(aux)

    def _move_left(self):
        aux = Twist()
        aux.angular.z = ANGULAR_VEL
        aux.linear.x = LINEAR_VEL / 10
        self._my_vel_pub.publish(aux)

    def _move_right(self):
        aux = Twist()
        aux.angular.z = -ANGULAR_VEL
        aux.linear.x = LINEAR_VEL / 10
        self._my_vel_pub.publish(aux)

    def move(self):

        if (self._is_left_detected and self._is_right_detected):
            self._move_forward()
            return
        if(self._is_left_detected):
            self._move_right()
            return
        if(self._is_right_detected):
            self._move_left()
            return
        self._move_forward()
        return
        

if __name__ == '__main__':
    node = LineFollowerNode()
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        node.move()
        rate.sleep()
