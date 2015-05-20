# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------
# Name:        Tracking
# Purpose:      Finds the a moving target between two laser scans
#
# Author:      Pablo Aponte
#              Ilya Mayugin
#
# Created:     19.10.2013
# Copyright:   (c) Pablo Aponte 2013
# Licence:     Pablo Aponte, Ilya Manyugin 2013
#-------------------------------------------------------------------------


import math
import cmath
import numpy
from scipy.spatial import distance
import icp
from config import *


class Tracker(object):
    """
    Intrudy Tracker class
    The class providing the target position. It localize a target when it starts to move and then track it as
    long as it is possible. When the target stops moving, the system tracks the target for a certain amount of
    time steps.
    """

    alpha = None  # minimum space between two closest points
    beta = None  # minimum number of points needed to consider movement
    gamma = None  # maximum space between points to be consider consecutive
    theta = None
    max_range = None  # max range of the laser
    previous_target = None  # previous target
    target = None  # current target
    mode = None  # current mode of the robot
    level = 0  # current sensibility level

    def __init__(self, mode):
        """ Constructor
        @param mode: the current mode of the robot
        """
        self.alpha = TrackingParameters.alpha
        self.beta = TrackingParameters.beta
        self.gamma = TrackingParameters.gamma
        self.theta = TrackingParameters.theta
        self.max_range = TrackingParameters.max_range
        self.level = TrackingParameters.initial_level
        self.mode = mode

    def polar2cart(self, laser, dist):
        """ Takes the laser number and distance to get the cartesian coordinates.
        @param laser: the laser number [0,540]
        @param dist:  the positive distance measurement for that laser
        """
        if laser == -1:
            return None
            # get the angle by subtracting 90 to the laser number since the first laser is -45° with a 0.5° step
        angle = (laser - 90) / 2
        x = math.cos(math.radians(angle)) * dist
        y = math.sin(math.radians(angle)) * dist
        return [x, y]

    def cart2polar(self, x, y):
        """ Takes the cartesian coordinates and change them to polar coordinates and then to
         the laser number and distance.
        @param x: coordinate x of the point in cartesian mode
        @param y: coordinate y of the point in cartesian mode
        """
        result = cmath.polar(complex(x, y))
        # Add 90 to obtain the laser number
        laser = abs(math.degrees(result[1]) * 2) + 90
        return [int(laser), result[0]]

    def truncate_scan(self, scan):
        """ Truncates the scan ranges bigger than the max range.
        @param scan: the array of scans that the function will truncate to max range
        """
        for i in range(len(scan)):
            # if the distance is bigger put the max range
            if scan[i] > self.max_range:
                scan[i] = self.max_range
        return scan

    def find_closest_point(self, target, newset):
        """ Find the closest point of each of the new point to the target set. It will return the points that have
        a closest point farther than alpha.
        @param target: the current set of scans
        @param newset: the new set of scans
        """
        target = numpy.array(target)
        newset = numpy.array(newset)
        # find the euclidean distance from each point of the target to the newset and then
        # get the index of those points with bigger distance than alpha
        ind = numpy.nonzero(
            numpy.all(distance.cdist(target, newset) > self.alpha[self.level], axis=0))
        # obtain the corresponding closest points from the newset
        points = newset[ind]
        outliers = zip(ind[0], points)
        return outliers

    def find_difference(self, target, newset, outliers):
        """ Find the difference between both scans for each of the outlier points. It will return a set of tuples with
        the difference and the point that represent a laser scan in cartesian mode.
        @param target: the current set of scans
        @param newset: the new set of scans
        @param outliers: the outlier points already found in the newset
        """
        result = []
        target = numpy.array(target)
        newset = numpy.array(newset)
        # get new distances that maybe were changed by ICP
        distance1 = distance.cdist(target, [[0, 0]])
        distance2 = distance.cdist(newset, [[0, 0]])
        for i in range(len(outliers)):
            # get the difference between scans in the outlier point (1st
            # derivative)
            temp = distance1[outliers[i][0]] - distance2[outliers[i][0]]
            result.append([temp, outliers[i][0]])
        return result

    def split_groups(self, lasers, scan2):
        """ Divide outlier points index into groups that contain consecutive indexes.
        @param lasers: the current set of tuples of outliers points and differences between the 2 scans
        """
        if not lasers:
            return []
        if len(lasers) == 1:
            return [lasers]
        ordered = sorted(lasers)
        temp = [ordered[0]]
        groups = []
        # group consecutive points that can have a separation no bigger than gamma
        for i in range(1, len(ordered)):
            if ordered[i] - ordered[i - 1] <= self.gamma[self.level] and abs(scan2[ordered[i]] - scan2[ordered[i-1]]) <= self.theta:
                temp.append(ordered[i])
            # if the separation is bigger finish the group and add it to the array and start a new group
            elif i != 0:
                groups.append(temp)
                temp = [ordered[i]]
        groups.append(temp)
        return groups

    def find_average(self, group):
        """ Find the distance average of a group of outlier points.
        @param group: the set of tuples that we want to find the average
        """
        average = []
        for g in group:
            # obtain sum of values and length of both parts of the group
            x = sum(g[0]) + sum(g[1])
            l = len(g[0]) + len(g[1])
            average.append(float(x) / float(l))
        return average

    def find_probable_target(self, positive, negative, scan2):
        """ Find the group of consecutive outlier points that is closer to the target.
        @param positive: the outliers with positive differences
        @param negative: the outliers with negative differences
        """
        # split the outliers with negative and positive difference in to groups of consecutive lasers
        temp_pos = self.split_groups(positive, scan2)
        temp_neg = self.split_groups(negative, scan2)
        groups = []
        # group positive and negative differences that are consecutive

        # if the positive difference group is empty then we get either a moving towards the target or none
        if len(positive) == 0:
            # if the negative difference group is also empty, we have no target
            if len(negative) == 0:
                return [[], []]
            # moving towards the robot
            else:
                groups = [[[], n] for n in temp_neg]
        # moving away from the robot
        elif len(negative) == 0:
            groups = [[p, []] for p in temp_pos]
            # other case (horizontal and diagonal movement of the target)
        else:
            for p in temp_pos:
                used = False
                for n in temp_neg:
                    if 0 <= p[0] - n[len(n) - 1] <= self.gamma[self.level] or 0 <= n[0] - p[len(p) - 1] <= self.gamma[
                        self.level]:
                        groups.append([p, n])
                        used = True
                        # if not used append it as an only positive difference
                if not used:
                    groups.append([p, []])
                    # find the only negative differences
            for n in temp_neg:
                used = False
                for p in temp_pos:
                    if  0 <= p[0] - n[len(n) - 1] <= self.gamma[self.level] or 0 <= n[0] - p[len(p) - 1] \
                            <= self.gamma[self.level]:
                        used = True
                    # add it to the group as only negative difference
                if not used:
                    groups.append([[], n])
        # choose from the groups the most probable target using the previous one as center

        # if previous target was None then choose the biggest one
        if self.previous_target is None:
            length = [len(e[0]) + len(e[1]) for e in groups]
            if len(length) == 0:
                return [[], []]
            else:
                best = numpy.argmax(length)
        # if previous target was set, center the averages to this point and find the closest group to it
        else:
            # find the average position of each group
            average = self.find_average(groups)
            if len(average) == 0:
                return [[], []]
            else:
                average = numpy.array(average) - self.previous_target[0]
                best = numpy.argmin(abs(average))
        return groups[best]

    def find_target(self, outliers_diff, scan2):
        """ Find the target using differences of distance between scans in the outlier points.
        The system, gives more importance to outliers closer to the previous target.
        @param outliers_diff: the set of tuples that contains the outlier points in cartesian coordinates and the
            difference between two scans
        """
        # obtain negative and positive differences
        positive = [e[1] for i, e in enumerate(outliers_diff) if e[0] > self.alpha[self.level]]
        negative = [e[1] for i, e in enumerate(outliers_diff) if e[0] < -self.alpha[self.level]]
        # find the most probable target
        positive, negative = self.find_probable_target(positive, negative, scan2)
        # no movement detected
        if len(positive) + len(negative) <= self.beta[self.level]:
            result = None
        elif len(positive) == 0:
            if len(negative) == 0:
                result = None
            else:
                # moving towards the robot
                result = sum(negative) / float(len(negative))
        else:
            # moving any other way
            result = sum(positive) / float(len(positive))
        return result

    def reduce_data(self, data1, data2):
        """  Reduce the data array to a window close to the previous target while the
        robot is not in Surveillance mode.
        @param data1: the first laser scans set in cartesian coordinates
        @param data2: the second laser scans set in cartesian coordinates
        """
        size = len(data1)
        window = 90
        data1 = numpy.array(data1)
        data2 = numpy.array(data2)
        # if it is not in surveillance mode, reduce the data
        if self.mode != IntrudyModes.SURVEILLANCE and self.previous_target is not None:
            # find the range of the window
            minimum = int(self.previous_target[0] - window)
            maximum = int(self.previous_target[0] + window + 1)
            # check for out of bounds
            if minimum <= 0:
                if maximum >= size:
                    return [data1, data2]
                else:
                    # set other points other than the window to [0,0]
                    new_data1 = numpy.concatenate((data1[minimum:maximum], [[0, 0]] * (size - maximum)), 0)
                    new_data2 = numpy.concatenate((data2[minimum:maximum], [[0, 0]] * (size - maximum)), 0)
            elif maximum >= size:
                # set other points other than the window to [0,0]
                new_data1 = numpy.concatenate(([[0, 0]] * minimum, data1[minimum:maximum]), 0)
                new_data2 = numpy.concatenate(([[0, 0]] * minimum, data2[minimum:maximum]), 0)
            else:
                # set other points other than the window to [0,0]
                new_data1 = numpy.concatenate(([[0, 0]] * minimum, data1[minimum:maximum], [[0, 0]] * (size - maximum)),
                                              0)
                new_data2 = numpy.concatenate(([[0, 0]] * minimum, data2[minimum:maximum], [[0, 0]] * (size - maximum)),
                                              0)
        else:
            return [data1, data2]
        return [new_data1, new_data2]

    def convert_data(self, scan1, scan2):
        """ Converts data to cartesian coordinates and truncates big values.
        @param scan1: the first raw laser scans set
        @param scan2: the second raw laser scans set
        """
        scan1 = self.truncate_scan(list(scan1))
        scan2 = self.truncate_scan(list(scan2))
        data1 = []
        i = 0
        for s in scan1:
            # change polar coordinates to cartesian
            data1.append(self.polar2cart(i, s))
            i += 1

        data2 = []
        i = 0
        for s in scan2:
            # change polar coordinates to cartesian
            data2.append(self.polar2cart(i, s))
            i += 1

        return [data1, data2]

    def change_level(self):
        """ Changes the sensibility level depending of the mode. """
        # if it is in surveillance mode, sensibility is higher
        if self.mode == IntrudyModes.SURVEILLANCE:
            self.level = 0
        # if it is in following or re-acquisition mode, sensibility is lower
        elif self.mode != IntrudyModes.SURVEILLANCE:
            self.level = 1

    def track(self, scan1, scan2, current_mode):
        """ Main function that finds the target based on movement detection using 2 scans.
        @param scan1: the first raw laser scans set
        @param scan2: the second raw laser scans set
        @param current_mode: the current mode of the robot
        """
        half = len(scan1)/2
        self.mode = current_mode
        self.change_level()
        # if collision was detected return None
        if self.mode == IntrudyModes.COLLISION:
            return None
            # convert to cartesian coordinates
        data1, data2 = self.convert_data(scan2, scan1)
        # optimize scan position using icp
        if self.mode != IntrudyModes.SURVEILLANCE:
            # change previous target to cartesian to move it using ICP if needed
            if self.previous_target is not None:
                past_target = self.polar2cart(self.previous_target[0], self.previous_target[1])
            else:
                past_target = None
                # do icp optimization
            icppoint = icp.Icp(data1, data2, TrackingParameters.icp_threshold, TrackingParameters.icp_max_iter,
                               past_target)
            icppoint.icp()
            data2 = icppoint.final_set
            # change target to the one optimized
            self.previous_target = self.cart2polar(icppoint.target_pos[0],
                                                   icppoint.target_pos[1]) if icppoint.target_pos is not None else None
        data2, data1 = self.reduce_data(data1, data2)
        # find outlier points and then the target
        outliers = self.find_closest_point(data1, data2)
        outliers_diff = self.find_difference(data1, data2, outliers)
        target = self.find_target(outliers_diff, scan2)
        # find target point in the scan
        if target is not None:
            target = int(math.floor(target))
            angle = (target - half) / 2
            self.previous_target = [target, scan2[target]]
            result = [math.radians(angle), scan2[target]]
        # if no movement detected and it is in Surveillance or reacquisition mode, return none
        elif self.mode == IntrudyModes.SURVEILLANCE:
            self.previous_target = None
            result = None
        elif self.mode == IntrudyModes.REACQUISITION:
            result = None
            # set current target our previous target for it to continue following it
            self.target = None if self.previous_target is None else self.previous_target[:]
            if self.target is not None:
                angle = (self.target[0] - half) / 2
                self.target[0] = math.radians(angle)
        else:
            result = None
            # update previous target
        return result