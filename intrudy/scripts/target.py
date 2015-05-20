#!/usr/bin/env python
# -*- coding: utf-8 -*-
#------------------------------------------------------------------------------
#       IntRudy - Intruder Detection and Following System
# Name:        Target robot
# Purpose:
#
# Author:      Ilya Manyugin
#              Pablo Aponte
#
# Created:     15.01.2014
# Copyright:   (c) Ilya Manyugin, Pablo Aponte, 2013
#------------------------------------------------------------------------------

import roslib
roslib.load_manifest('intrudy')
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from record import Record


class Target(object):
    """ Target main class """
    command = Twist()
    scan_data = None
    recorder = None

    def __init__(self):
        """ Constructor """
        # init ROS node
        rospy.init_node('Target')
        # Laser Scanner
        self.ls = rospy.Subscriber('laserscan', LaserScan, self.laser_callback)
        # Command publisher
        self.comp = rospy.Publisher('cmd_vel', Twist)
        # Recording module
        self.recorder = Record("logs", "log.txt", "scans", "mode_log.txt")

    def laser_callback(self, data):
        """ Laser callback called by the subscriber """
        self.scan_data = data
        self.recorder.save_scan(data)

    def emergency_stop(self):
        """ Detect whether any object is too close to the robot """
        if self.scan_data is None:
            return  True
        for rng in self.scan_data.ranges:
            if rng <= 0.25:
                return True
        return False

    def calculate_command(self):
        if self.emergency_stop():
            # stop the robot
            self.command.linear.x = 0.
            self.command.angular.z = 0.
        else:
            #move forward
            self.command.linear.x = 0.2
            self.command.angular.z = 0.

    def main_loop(self):
        """ Main execution loop """
        #set the rospy rate to 20 Hz
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.calculate_command()
            self.comp.publish(self.command)
            r.sleep()


if __name__ == '__main__':
    try:
        target = Target()
        target.main_loop()
    except rospy.ROSInterruptException as e:
        print e
