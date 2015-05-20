#------------------------------------------------------------------------------
#       IntRudy - Intruder Detection and Following System
# Name:        Tracking Parameters
# Purpose:
#
# Author:      Ilya Manyugin
#              Pablo Aponte
#
# Created:     10.12.2013
# Copyright:   (c) Ilya Manyugin, Pablo Aponte, 2013
#------------------------------------------------------------------------------


class TrackingParameters:
    """ TrackingParameters: a class holding the parameters for the Intrudy System """

    max_range = 3  # max range of the laser

    # sensibilty levels from highest to lowest
    # alpha: minimum space between two closest points
    # beta: minimum number of points needed to consider movement
    # gamma: maximum space between points to be consider consecutive
    alpha = [0.03, 0.04]
    beta = [7, 15]
    gamma = [3, 1]
    theta = 0.5
    initial_level = 0 # initial sensibilty level

    #ICP parameters used in the tracking module
    icp_threshold = 0.001 # maximum MSE
    icp_max_iter = 10 # maximum number of iterations


class IntrudyConstants:
    """ IntrudyConstants: a class holding the constants for the Intrudy System """
    min_lin_speed = 0.1
    min_ang_speed = 0.1
    max_lin_speed = 0.2
    max_ang_speed = 0.2
    update_period = None
    reacquisiton_threshold = 20
    test_mode = False
    skip_rate = 5


class IntrudyModes:
    """ States of the FSM """
    SURVEILLANCE = 0
    FOLLOWING = 1
    REACQUISITION = 2
    COLLISION = 3

    names = {0:"SURVEILLANCE", 1:"FOLLOWING", 2:"REACQUISITION", 3:"COLLISION"}