#------------------------------------------------------------------------------
#       IntRudy - Intruder Detection and Following System
# Name:        Recording Module
# Purpose:     Module used for recording the testing parameters
#
# Author:      Ilya Manyugin
#              Pablo Aponte
#
# Created:     14.01.2014
# Copyright:   (c) Ilya Manyugin, Pablo Aponte, 2014
#------------------------------------------------------------------------------

import os
import errno
import time
from config import *


class Record:

    log_file = None
    mode_file = None
    scans_folder = None
    log_folder = None
    current_scan = 0

    def __init__(self, log_folder, log_file, scans_folder, mode_log_file):
        """
        Constructor
        @param log_folder: Folder where the logs are going to be saved
        @param log_file: name of the log file
        @param scans_folder: folder where the scans are going to be
        @param mode_log_file: name of the log file with the success cases for each mode
        """
        self.log_folder = log_folder
        self.log_file = log_file
        self.scans_folder = scans_folder
        self.mode_file = mode_log_file

        # make sure the directory exist, if not create it
        self.create_dir(self.log_folder)
        self.create_dir(self.scans_folder)

    def create_dir(self, folder):
        """

        @param folder:
        @raise:
        """
        script_dir = os.path.dirname(__file__)
        path = os.path.join(script_dir, folder)
        try:
            os.makedirs(path)
        except OSError as exception:
            if exception.errno != errno.EEXIST:
                raise

    def save_scan(self, scan):
        """
        Save the scan in to a file to be able to repeat the experiments.
        @param scan: the complete laser scan
        """
        # change the scan number
        self.current_scan += 1

        # get the path to the file
        filename = "scan_{0:04d}.txt".format(self.current_scan)
        script_dir = os.path.dirname(__file__)
        abs_file_path = os.path.join(script_dir, self.scans_folder, filename)

        # open the file
        f = open(abs_file_path, 'w')

        #save the scan
        for i, s in enumerate(scan.ranges):
            f.write("{0} {1}\n".format(i, s))
        # close the file
        f.close()

    def add_to_log(self, text, log_type=0):
        # get the path to the file
        """
        Add a timestamped log in the log file.
        @param text: the text that goes after the timestamp
        @param log_type: the type of log, currently unused
        """
        script_dir = os.path.dirname(__file__)
        abs_file_path = os.path.join(script_dir, self.log_folder, self.log_file)

        # open the file
        f = open(abs_file_path, 'a+')

        # get timestamp
        timestamp = str(time.time())

        #save the log
        f.write(timestamp + " " + text + "\n")
        f.close()

    def add_mode_status(self, value, mode):
        # get the path to the file
        """
        Logs the success or failure of detecting a target for each state
        @param value: text with Success or Fail
        @param mode: the current mode of the system
        """
        script_dir = os.path.dirname(__file__)
        abs_file_path = os.path.join(script_dir, self.log_folder, self.mode_file)

        # open the file
        f = open(abs_file_path, 'a+')

        # get timestamp
        timestamp = str(time.time())

        #save the log
        f.write(timestamp + " " + IntrudyModes.names[mode] + " " + value + "\n")
        f.close()