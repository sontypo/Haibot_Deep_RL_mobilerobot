#!/usr/bin/env python3

import rospy
import numpy as np
import math
from math import *
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Env():
    def __init__(self, action_size):
        self.init_x = 0.0 #m
        self.init_y = 0.0
        self.goal_x = 0.5  #| fixed goal
        self.goal_y = 0.0   #|
        self.heading = 0
        self.action_size = action_size
        self.num_scan_ranges = 20
        self.initGoal = True
        self.get_goal = False
        self.prev_distance = 0
        self.const_vel = 0.225    #0.25

        self.current_theta = 0
        self.goal_counters = 0
        self.enable_feedback_control = False
        self.safe_dist = 1.0
        self.lidar = []
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.list_goal_x = [0.5, 1.3, 2.0, 2.0, 3.2]
        self.list_goal_y = [0.0, -1.0, 0.4, 1.5, -2.5]
        
    
    def getGoalDistance(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, cur_theta = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - cur_theta
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)
        self.current_theta = cur_theta #radian
        
        return self.position.x, self.position.y, self.current_theta

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.35
        done = False
        
        # Formatting the scan_range to feed algorithm
        # scan_range.append(max(scan[8], scan[9], scan[10]))
        # scan_range.append(max(scan[45], scan[46], scan[47]))
        # scan_range.append(max(scan[82], scan[83], scan[84]))
        # scan_range.append(max(scan[119], scan[120], scan[121]))
        # scan_range.append(max(scan[156], scan[157], scan[158]))
        # scan_range.append(max(scan[193], scan[194], scan[195]))
        # scan_range.append(max(scan[230], scan[231], scan[232]))
        # scan_range.append(max(scan[267], scan[268], scan[269]))
        # scan_range.append(max(scan[304], scan[305], scan[306]))
        # scan_range.append(max(scan[341], scan[342], scan[343]))
        # scan_range.append(max(scan[378], scan[379], scan[380]))
        # scan_range.append(max(scan[415], scan[416], scan[417]))
        # scan_range.append(max(scan[452], scan[453], scan[454]))
        # scan_range.append(max(scan[489], scan[490], scan[491]))
        # scan_range.append(max(scan[526], scan[527], scan[528]))
        # scan_range.append(max(scan[563], scan[564], scan[565]))
        # scan_range.append(max(scan[600], scan[601], scan[602]))
        # scan_range.append(max(scan[637], scan[638], scan[639]))
        # scan_range.append(max(scan[674], scan[675], scan[676]))
        # scan_range.append(max(scan[711], scan[712], scan[713]))
        scan_range.append(scan.ranges[9])
        scan_range.append(scan.ranges[46])
        scan_range.append(scan.ranges[83])
        scan_range.append(scan.ranges[120])
        scan_range.append(scan.ranges[157])
        scan_range.append(scan.ranges[194])
        scan_range.append(scan.ranges[231])
        scan_range.append(scan.ranges[268])
        scan_range.append(scan.ranges[305])
        scan_range.append(scan.ranges[342])
        scan_range.append(scan.ranges[379])
        scan_range.append(scan.ranges[416])
        scan_range.append(scan.ranges[453])
        scan_range.append(scan.ranges[490])
        scan_range.append(scan.ranges[527])
        scan_range.append(scan.ranges[564])
        scan_range.append(scan.ranges[601])
        scan_range.append(scan.ranges[638])
        scan_range.append(scan.ranges[675])
        scan_range.append(scan.ranges[712])
        
        # Other way to format scan_range
        # cof = (len(scan.ranges) / (self.num_scan_ranges - 1))
        # for i in range(0, self.num_scan_ranges):
        #     n_i = math.ceil(i*cof - 1)
        #     if n_i < 0:
        #         n_i = 0
        #     if cof == 1:
        #         n_i = i
        #     if scan.ranges[n_i] == float('Inf'):
        #         scan_range.append(3.5)
        #     elif np.isnan(scan.ranges[n_i]):
        #         scan_range.append(0)
        #     else:
        #         scan_range.append(scan.ranges[n_i])
        
        for i in range(len(scan_range)):
            if scan_range[i] == float('Inf'):
                scan_range[i] = 3.5
            elif np.isnan(scan_range[i]):
                scan_range = 0.0
            scan_range[i] = round(scan_range[i], 4)

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance <= 0.35:
            self.get_goal = True

        return scan_range + [heading, current_distance], done

    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-1]
        heading = state[-2]

        for i in range(self.action_size):
            angle = -pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (current_distance / self.goal_distance)
        reward = ((round(yaw_reward[action] * self.action_size, 2)) * distance_rate)
        
        dist_rate = self.prev_distance - current_distance
        self.prev_distance = current_distance

        if done:
            rospy.loginfo("*****************")
            rospy.loginfo("* COLLISION !!! *")
            rospy.loginfo("*****************")
            reward = -650.
            self.pub_cmd_vel.publish(Twist())
            rospy.spin(5)
        
        elif dist_rate > 0:
            reward = 200.*dist_rate
        
        elif dist_rate <= 0:
            reward = -8.
        
        # # Reward for Feedback control status:
        # elif self.enable_feedback_control:
        #     reward = -100
        # elif not self.enable_feedback_control:
        #     reward = 0
        

        if (self.get_goal):
            rospy.loginfo("********************")
            rospy.loginfo("* GOAL REACHED !!! *")
            rospy.loginfo("********************")
            reward = 550.
            self.goal_counters += 1
            self.pub_cmd_vel.publish(Twist())
            
            # if self.position.x == self.init_x and self.position.y == self.init_y:   
            self.goal_x = self.list_goal_x[self.goal_counters]
            self.goal_y = self.list_goal_y[self.goal_counters]
            self.goal_distance = self.getGoalDistance()
            self.get_goal = False
            if self.goal_counters >5:
                self.goal_counters = 0

        return reward, self.goal_counters

    ###############################################################################################################
    # def FeedBackControl(self, odom):
    #     theta_goal = np.random.uniform(0, (pi*2))
    #     x, y, theta = self.getOdometry(odom)
        
    #     if theta_goal >= pi:
    #         theta_goal_norm = theta_goal - 2 * pi
    #     else:
    #         theta_goal_norm = theta_goal
        
    #     ro = sqrt( pow( ( self.goal_x - x ) , 2 ) + pow( ( self.goal_y - y ) , 2) )
    #     lamda = atan2( self.goal_y - y , self.goal_x - x )
        
    #     # print(" x_goal = {:.2f}, y_goal = {:.2f}".format(self.goal_x, self.goal_y))

    #     alpha = (lamda -  theta + pi) % (2 * pi) - pi
    #     beta = (theta_goal - lamda + pi) % (2 * pi) - pi

    #     if ro < self.goal_dist_thres and degrees(abs(theta-theta_goal_norm)) < self.goal_angle_thres:
    #         status = '--> (Feedback) Goal position reached !!! '
    #         v = 0
    #         w = 0
    #         v_scal = 0
    #         w_scal = 0
    #     else:
    #         status = '--> (Feedback) Go to the destination ... '
    #         v = self.k_r * ro
    #         w = self.k_alpha * alpha + self.k_beta * beta
    #         v_scal = v / abs(v) * self.const_vel
    #         w_scal = w / abs(v) * self.const_vel

    #     vel_cmd = Twist()
    #     vel_cmd.linear.x = v_scal
    #     vel_cmd.angular.z = w_scal
    #     self.pub_cmd_vel.publish(vel_cmd)

    #     return status
    ###############################################################################################################

    def step(self, action):
        data = None
        odom = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan)
                odom = rospy.wait_for_message('/odom', Odometry)
            except:
                pass
            
        state, done = self.getState(data)
            
        max_angular_vel = 0.75  #1.5 0.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = self.const_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)
            
        # # Switching Algorithms:
        # if min(state[:20]) >= self.safe_dist:
        #     status = self.FeedBackControl(odom)
        #     self.enable_feedback_control = True
        # else:
        #     self.enable_feedback_control = False       
            
        reward, counters = self.setReward(state, done, action)

        return np.asarray(state), reward, done, counters

    def reset(self):

        data = None
        odom = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan)
                odom = rospy.wait_for_message('/odom', Odometry)
            except:
                pass

        self.init_x, self.init_y, self.current_theta = self.getOdometry(odom)
        
        self.goal_distance = self.getGoalDistance()
        state, done = self.getState(data)
        self.goal_counters = 0
        self.goal_x = self.list_goal_x[self.goal_counters]
        self.goal_y = self.list_goal_y[self.goal_counters]
        self.lidar = state

        return np.asarray(state)