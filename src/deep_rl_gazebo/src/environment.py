#!/usr/bin/env python3

import rospy
import numpy as np
import math
from math import *
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from src.respawnGoal import Respawn

class Env():
    def __init__(self, action_size):
        self.init_x = -2.3 #m
        self.init_y = -1.0
        # self.goal_x = -1.0   #| random goal
        # self.goal_y = 0.0    #|
        self.goal_x = 1.65  #| fixed goal
        self.goal_y = 2.0   #|
        self.heading = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.prev_distance = 0
        # self.k_r = 2 
        # self.k_alpha = 15 
        # self.k_beta = -3
        self.const_vel = 0.225    #0.25
        # self.goal_dist_thres = 0.2  #0.55
        # self.goal_angle_thres = 15 #degrees
        self.current_theta = 0
        self.goal_counters = 0
        self.enable_feedback_control = False
        self.safe_dist = 1.0
        self.lidar = []
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        
    
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

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan.ranges[i])

        if min_range > min(scan_range) > 0:
            done = True

        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y),2)
        if current_distance <= 0.35:
            self.get_goalbox = True

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
        # print("Distance rate: %f" % dist_rate)

        if done:
            rospy.loginfo("*****************")
            rospy.loginfo("* COLLISION !!! *")
            rospy.loginfo("*****************")
            reward = -650.
            self.pub_cmd_vel.publish(Twist())
        
        elif dist_rate > 0:
            reward = 200.*dist_rate
        
        elif dist_rate <= 0:
            reward = -8.
        
        # # Reward for Feedback control status:
        # elif self.enable_feedback_control:
        #     reward = -100
        # elif not self.enable_feedback_control:
        #     reward = 0
        

        if (self.get_goalbox):
            rospy.loginfo("********************")
            rospy.loginfo("* GOAL REACHED !!! *")
            rospy.loginfo("********************")
            reward = 550.
            self.goal_counters += 1
            self.pub_cmd_vel.publish(Twist())
            # self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            
            # if self.position.x == self.init_x and self.position.y == self.init_y:   
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(False, delete=True)
            self.goal_distance = self.getGoalDistance()
                # self.get_goalbox = False
            self.get_goalbox = False
            rospy.wait_for_service('gazebo/reset_simulation')
            try:
                self.reset_proxy()
            except (rospy.ServiceException) as e:
                print("gazebo/reset_simulation service call failed")

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
        max_angular_vel = 0.75  #1.5 0.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()
        vel_cmd.linear.x = self.const_vel
        vel_cmd.angular.z = ang_vel
        self.pub_cmd_vel.publish(vel_cmd)

        data = None
        odom = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan)
                odom = rospy.wait_for_message('/odom', Odometry)
            except:
                pass
            
        state, done = self.getState(data)
        
        # # Switching Algorithms:
        # if min(state[:20]) >= self.safe_dist:
        #     status = self.FeedBackControl(odom)
        #     self.enable_feedback_control = True
        # else:
        #     self.enable_feedback_control = False       
            
        reward, counters = self.setReward(state, done, action)

        return np.asarray(state), reward, done, counters

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        odom = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan)
                odom = rospy.wait_for_message('/odom', Odometry)
            except:
                pass

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False

        self.init_x, self.init_y, self.current_theta = self.getOdometry(odom)
        
        self.goal_distance = self.getGoalDistance()
        state, done = self.getState(data)
        self.goal_counters = 0
        # self.lidar = state

        return np.asarray(state)