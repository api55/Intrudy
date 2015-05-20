#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------------------------------
#       IntRudy - Intruder Detection and Following System
# Name:        Main executable script
# Purpose:
#
# Author:      Ilya Manyugin
#              Pablo Aponte
#
# Created:     30.06.2013
# Copyright:   (c) Ilya Manyugin, Pablo Aponte, 2013
#------------------------------------------------------------------------------

import roslib
roslib.load_manifest('intrudy')
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from controller import IntrudyController


class IntRudy(object):

    """ Intrudy main class """
    roomba_command = Twist()
    LOG = True

    def __init__(self):
        """ Constructor """
        # init ROS node
        rospy.init_node('IntRudy')
        # Laser Scanner
        self.ls = rospy.Subscriber('laserscan', LaserScan, self.laser_callback)
        # Command publisher
        self.comp = rospy.Publisher('cmd_vel', Twist)
        self.controller = IntrudyController(self.comp)

    def laser_callback(self, data):
        """ Laser callback called by the subscriber """
        self.controller.sensor_input(data)

    def main_loop(self):
        """ Main execution loop """
        #set the rospy rate to 20 Hz
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.controller.calculate_command(self.roomba_command)
            self.comp.publish(self.roomba_command)
            rospy.loginfo(
                '\tCommand: x={0.linear.x: .2f}, z={0.angular.z: .2f}'.format(self.roomba_command))
            r.sleep()


if __name__ == '__main__':
    try:
        rudy = IntRudy()
        rudy.main_loop()
    except rospy.ROSInterruptException as e:
        print e
