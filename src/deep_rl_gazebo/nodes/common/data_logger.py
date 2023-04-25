#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import argparse
import sys
import os
from datetime import datetime

class DataLogger():

    def __init__(self):
            
        # variables
        self.x = []
        self.y = []
        self.theta = []
        self.v = []  
        self.omega = [] 
        self.min_dist = []
        self.object_id = ["diff_robot"]

        # define path
        self.directory = os.path.dirname(os.path.abspath(__file__))+'/logs/'

    def store_data(self):
        # get one instance of message
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=3)
                scan_data = rospy.wait_for_message('scan', LaserScan, timeout=3)
            except:
                pass

        # Find the index of this object_id in the name list:
        idx = 6
        # Retrieve states from data
        self.x.append(data.pose[idx].position.x)
        self.y.append(data.pose[idx].position.y)
        self.v.append(data.twist[idx].linear.x)
        self.omega.append(data.twist[idx].angular.z)

        # obtain min obstacle distance
        scan_range = []
        for i in range(len(scan_data.ranges)):
            if scan_data.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan_data.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan_data.ranges[i])

        self.min_dist.append(round(min(scan_range), 2))

    def clear_data(self):
        self.x = []
        self.y = []
        self.theta = []
        self.v = []  
        self.omega = [] 
        self.min_dist = []


    def save_data(self, trial, done='succeed'):
        # save state data
        data = np.array([done, self.x, self.y, self.v, self.omega, self.min_dist])
        
        filename = 'test_'+str(trial)
        if os.path.isdir(self.directory):
            np.save(self.directory+filename+".npy", data)
        else:
            os.makedirs(self.directory)
            np.save(self.directory+filename+".npy", data)

        # clear data
        self.clear_data()