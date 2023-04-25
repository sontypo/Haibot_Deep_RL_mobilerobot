#!usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from math import *
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

DATA_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = DATA_PATH.replace('dqn_gazebo/nodes', 'dqn_gazebo/save_model/PT_trial_with_fixed_goal_1/log_data')

def plotting_data(load_dir):
    reward_per_episode = np.genfromtxt(load_dir+'/reward_per_episode.csv', delimiter = ' , ')
    steps_per_episode = np.genfromtxt(load_dir+'/steps_per_episode.csv', delimiter = ' , ')
    loss_per_epoch = np.genfromtxt(load_dir+'/loss.csv', delimiter = ' , ')
    goal_counter_per_episode = np.genfromtxt(load_dir+'/goal_counters_per_episode.csv', delimiter = ' , ')
    
    accumulated_reward = np.array([])
    average_steps_per10eps = np.array([])
    episodes_10th = np.arange(10, len(reward_per_episode)+10, 10)
    
    for i in range(len(episodes_10th)):
        accumulated_reward = np.append(accumulated_reward, np.sum(reward_per_episode[0:10*(i+1)]))
        average_steps_per10eps = np.append(average_steps_per10eps, np.sum(steps_per_episode[10*i:10*(i+1)]) / 10)
        
    plt.style.use('seaborn-ticks')

    ## plotting reward per episode
    plt.figure(1)
    plt.plot(reward_per_episode, color='teal')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.title('Total reward per episode')
    plt.grid()
    
    ## plotting accumulated reward through 10 episodes
    plt.figure(2)
    plt.plot(episodes_10th, accumulated_reward, color='teal')
    plt.xlabel('Episode')
    plt.ylabel('Accumulated reward')
    plt.title('Accumulated reward per 10 episodes')
    plt.ylim(np.min(accumulated_reward) - 1000 , np.max(accumulated_reward) + 1000)
    # plt.xlim(np.min(episodes_10th), np.max(episodes_10th))
    plt.grid()
    
    ## plotting steps per episode
    plt.figure(4)
    plt.plot(steps_per_episode, color='darkslateblue')
    plt.xlabel('Episode')
    plt.ylabel('Steps')
    plt.title('Steps per episode')
    plt.ylim(np.min(steps_per_episode) - 10, np.max(steps_per_episode) + 10)
    plt.grid()
    
    ## plotting losses over epochs
    plt.figure(5)
    plt.plot(loss_per_epoch, color='darkviolet')
    plt.xlabel('Episode')
    plt.ylabel('Losses')
    plt.title('Losses over epochs')
    plt.grid()
    
    ## plotting goal counter per episode
    plt.figure(6)
    plt.plot(goal_counter_per_episode, color='darkblue')
    plt.xlabel('Episode')
    plt.ylabel('Goal counts')
    plt.title('Goal counter per episode')
    plt.grid()
    
    plt.tight_layout()
    plt.show()
    
if __name__ == '__main__':
    try:
        print("** PLOTTING DATA ***")
        plotting_data(DATA_PATH)
    except rospy.ROSInterruptException:
        print("--> Exiting !!!")
        pass