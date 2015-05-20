# -*- coding: utf-8 -*-
#-------------------------------------------------------------------------
#       IntRudy - Intruder Detection and Following System
# Name:        ICP
# Purpose:     icp in 2D done two find concordance between 2 laser scans
#
# Author:      Pablo Aponte
#              Ilya Mayugin
#
# Created:     16.10.2013
# Copyright:   (c) Pablo Aponte, Ilya Manyugin 2013
#-------------------------------------------------------------------------

import math
import numpy
from scipy.spatial import distance


class Icp(object):
    """ ICP class used for finding the correspondence of one scan set with another one to eliminate the movement
     in scans done by the movement of the robot."""
    target = numpy.array([])  # target set (old scan)
    newset = numpy.array([])  # new set (new scan)
    correspondence = []  # set of corresponding points of the target set for the new set
    correlation = numpy.array([])  # correlation matrix
    target_mean = numpy.array([])  # target set mean
    newset_mean = numpy.array([])  # new set mean
    rotation = numpy.array([[1, 0], [0, 1]])  # current rotation matrix (changes per iteration)
    final_set = numpy.array([])  # new set of point with the rotation and translation applied
    threshold = 10  # minimum mse permitted before ending
    max_iter = 10  # maximum iterations permitted
    target_position = None

    def __init__(self, target, newset, threshold, max_iter, target_pos):
        """ Constructor
        @param target: the target scan set
        @param newset: the new set that we want to find correspondence
        @param threshold: max mse allowed
        @param max_iter: max iterations for the icp loop
        @param target_pos: current target position to optimize it using icp
        """
        # change lists to numpy array to do matrix operations faster
        self.target = numpy.array(target)
        self.newset = numpy.array(newset)
        self.final_set = numpy.array(newset)
        self.threshold = threshold
        self.max_iter = max_iter
        if target_pos is not None:
            self.target_pos = numpy.array(target_pos)
        else:
            self.target_pos = None

    def mse(self):
        """ Calculates the mean square error"""
        errors = self.correspondence - self.newset
        errors = [math.pow(x[0], 2) + math.pow(x[1], 2) for x in errors]
        return sum(errors) / float(len(errors))

    def find_closest_point(self):
        """ Find the closest point of each of the new point to the target set """
        self.correspondence = []
        # get all the euclidean distances
        dists = distance.cdist(self.target, self.final_set)
        # get the index of the minimum distances
        minid = numpy.argmin(dists, axis=0)
        # get the points in the other set that has the minimum distances
        self.correspondence = self.target[minid]
        # remove outlier points
        ind = numpy.nonzero(numpy.any(dists <= 0.5, axis=0))
        self.correspondence = self.correspondence[ind]
        self.newset = self.final_set[ind]

    def find_correlation(self):
        """ find the correlation matrix """
        # calculate mean point in target and newset
        self.target_mean = sum(self.correspondence) / float(
            len(self.correspondence))
        self.newset_mean = sum(self.newset) / float(len(self.newset))
        # each point substract the mean of the data set
        temp_target = self.correspondence - self.target_mean
        temp_newset = self.newset - self.newset_mean
        # get the correlation matrix
        temp_target = numpy.array([temp_target[:, 0], temp_target[:, 1]])
        temp_newset = numpy.transpose(
            numpy.array([temp_newset[:, 0], temp_newset[:, 1]]))
        self.correlation = numpy.dot(temp_target, temp_newset)

    def find_RT(self):
        """ Find the rotation matrix and translation vector """
        # apply SVD algorithm to obtain U,V for the rotation matrix
        [U, D, V] = numpy.linalg.svd(self.correlation)
        self.rotation = numpy.dot(U, numpy.transpose(V))
        self.translation = self.target_mean - \
            numpy.dot(self.rotation, self.newset_mean)

    def RT_point(self, point):
        """ Apply rotation and translation to a point """
        return numpy.dot(self.rotation, point) + self.translation

    def apply_RT(self):
        """ Apply rotation and translation to newset """
        self.final_set = numpy.array(map(self.RT_point, self.final_set))
        # apply it also to the target point
        if self.target_pos is not None:
            self.target_pos = self.RT_point(self.target_pos)

    def icp(self):
        """
        Main icp routine that finds the rotation and translation matrix and applies it
        to the set of points until convergence.
        """
        mse_value = float('inf')
        i = 0
        # until convergence
        while mse_value > self.threshold and i < self.max_iter:
            i += 1
            # find the correspondence and correlation matrix
            self.find_closest_point()
            self.find_correlation()
            # find the rotation and translation of this iteration and apply it
            self.find_RT()
            self.apply_RT()
            # get the new value of the mse
            mse_value = self.mse()
