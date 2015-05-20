# -*- coding: utf-8 -*-
#------------------------------------------------------------------------------
#       IntRudy - Intruder Detection and Following System
# Name:        Intrudy Controller
# Purpose:
#
# Author:      Ilya Manyugin
#              Pablo Aponte
#
# Created:     30.06.2013
# Copyright:   (c) Ilya Manyugin, Pablo Aponte, 2013
#------------------------------------------------------------------------------

import math

from geometry_msgs.msg import Twist
from tracking import Tracker
from config import *
from record import Record
import rospy


def debug_info(msg):
    """ Debug message function """
    rospy.loginfo(msg)


def enum(**enums):
    """ A function creating new type alike to the enum in C """
    return type('Enum', (), enums)


class IntrudyController(object):
    """Intrudy Controller class
    The class providing the FSM for the robot state tracking and transition, issuing commands to the motors
     """

    max_scans = 2
    skip_rate = None  # taken from the config
    scan_data = []
    current_stamp = None
    input_seq_num = 0
    current_mode = IntrudyModes.SURVEILLANCE
    reacquisition_step = 8
    last_target = None
    recorder = None

    def __init__(self, publisher):
        """ Constructor """
        super(IntrudyController, self).__init__()
        self.skip_rate = IntrudyConstants.skip_rate
        self.publisher = publisher
        self.tracker = Tracker(self.current_mode)
        if IntrudyConstants.test_mode:
            self.recorder = Record("logs", "log.txt", "scans", "mode_log.txt")

    def locate_target(self):
        """ Locate the target
        A proxy call to a separate module.
        Valid return values: None or a two-tuple (angle, distance)
        """
        if IntrudyConstants.test_mode:
            self.recorder.save_scan(self.scan_data[-1])
        return self.tracker.track(self.scan_data[-2].ranges,
                                  self.scan_data[-1].ranges,
                                  self.current_mode)

    def avoid_obstacles(self, command):
        """ Correct the command in order to avoid obstacles """
        if self.detect_collision():
            command.linear.x = 0.
            command.angular.z = IntrudyConstants.max_ang_speed / 1.5
        else:
            debug_info('Collision Avoided. Switching to SURVEILLANCE MODE.')
            command.linear.x = 0.
            command.angular.z = 0.
            self.current_mode = IntrudyModes.SURVEILLANCE
            self.scan_data = []
            self.input_seq_num = -5
            self.reacquisition_step = 0

    def drive_towards(self, target, command):
        """ Set the command to steer towards the target """
        # if we detect a collision, stop immediately
        if self.detect_collision():
            command.linear.x = 0.
            command.angular.z = 0.
            self.current_mode = IntrudyModes.COLLISION
            debug_info('! Collision has been detected')
            return

        angle, distance = target
        if distance > 2:
            command.linear.x = IntrudyConstants.max_lin_speed
        else:
            command.linear.x = IntrudyConstants.max_lin_speed * 2 / 3
        sign = math.copysign(1, angle)
        if abs(angle) > IntrudyConstants.max_ang_speed * 3:
            command.angular.z = IntrudyConstants.max_ang_speed * sign
        elif abs(angle) < IntrudyConstants.min_ang_speed and abs(angle) != 0:
            command.angular.z = IntrudyConstants.min_ang_speed * sign
        else:
            command.angular.z = 0.

    def detect_collision(self):
        """ Detect whether any object is too close to the robot """
        for i in range(90,450):
            if self.scan_data[-1].ranges[i] <= 0.25:
                return True
        return False

    def sensor_input(self, data):
        """ Input from the laser scanner """
        self.input_seq_num += 1
        if self.input_seq_num != self.skip_rate:
            return
        else:
            self.input_seq_num = 0
        self.scan_data.append(data)
        # keep the last max_scans sensor readings
        while len(self.scan_data) > self.max_scans:
            del self.scan_data[0]
        self.current_stamp = data.header

    def calculate_command(self, command):
        """ Calculate the command for the robot """

        if self.current_mode == IntrudyModes.COLLISION:
            self.avoid_obstacles(command)
            return

        if len(self.scan_data) < 2:
            return

        # locate target and follow it
        target = self.locate_target()
        if target is not None:
            if IntrudyConstants.test_mode:
                self.recorder.add_mode_status('Success', self.current_mode)
                self.recorder.add_to_log('Target located @ {0[0]} radians, {0[1]} meters.'.format(
                    target))
            debug_info('Target located @ {0[0]} radians, {0[1]} meters.'.format(
                target))
            self.last_target = target
            if self.current_mode == IntrudyModes.SURVEILLANCE or self.current_mode == IntrudyModes.REACQUISITION:
                self.current_mode = IntrudyModes.FOLLOWING
                self.reacquisition_step = 0
            self.drive_towards(target, command)

        else:
            if IntrudyConstants.test_mode:
                self.recorder.add_mode_status('Fail', self.current_mode)
            if self.current_mode == IntrudyModes.FOLLOWING:
                debug_info('Target lost. Switching to RE-ACQUISITION MODE.')
                self.current_mode = IntrudyModes.REACQUISITION
                self.drive_towards(self.last_target, command)
                self.reacquisition_step += 1
            if self.current_mode == IntrudyModes.REACQUISITION:
                debug_info('Re-acquisition step is {}.'.format(self.reacquisition_step))
                self.last_target = self.tracker.target
                if self.last_target is not None:
                    self.drive_towards(self.last_target, command)
                    # if the target is too close to the robot
                    if self.tracker.target[1] <= 0.6:
                        command.linear.x = 0.
                self.reacquisition_step += 1
                if self.reacquisition_step > IntrudyConstants.reacquisiton_threshold:
                    debug_info('Re-acquisition failed. Switching to SURVEILLANCE MODE.')
                    self.reacquisition_step = 0
                    self.current_mode = IntrudyModes.SURVEILLANCE
                    self.scan_data = []
                    self.input_seq_num = -5
            if self.current_mode == IntrudyModes.SURVEILLANCE:
                command.linear.x = 0.
                command.angular.z = 0.

