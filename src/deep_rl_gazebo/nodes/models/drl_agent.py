#!usr/bin/env python3

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torch.nn.init as init
import rospy
import random
import numpy as np
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from collections import deque
from std_msgs.msg import Float32MultiArray
from nodes.models.q_network import QNetwork, DuelingQNetwork
from nodes.common.replay_buffers import NormalBuffer, PrioritizedBuffer

    
class DeepReinforceAgent():
    def __init__(self, state_size, action_size, policy_type, net_type, using_per, run_mode, load_eps, save_dir):
        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
        self.dirPath = os.path.dirname(os.path.abspath(os.path.abspath(os.path.realpath(__file__))))
        self.save_path = self.dirPath.replace('deep_rl_gazebo/nodes/models', save_dir + '/pt_trial_1_')
        self.loss_dir = self.save_path.replace('deep_rl_gazebo/nodes/models', save_dir + '/log_data')
        self.result = Float32MultiArray()
        self.policy_type = policy_type
        self.net_type = net_type
        self.using_per = using_per
        self.run_mode = run_mode
        self.load_model = False
        self.load_episode = load_eps
        self.state_size = state_size
        self.action_size = action_size
        self.episode_step = 6000
        self.target_update = 2000   #2000    # after target_update's time, Target network will be updated
        self.discount_factor = 0.995
        self.learning_rate = 0.001      #0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.997      #0.996
        self.epsilon_min = 0.01
        self.batch_size = 64     
        self.train_start = 64
        self.memory_size = 1000000
        
        ## Selecting specific network type and applying for each RL Agent
        if self.net_type == "dueling" or self.net_type == "d3q":
            self.Pred_model = DuelingQNetwork(self.state_size, self.action_size)
            self.Target_model = DuelingQNetwork(self.state_size, self.action_size)
        else:
            self.Pred_model = QNetwork(self.state_size, self.action_size)
            self.Target_model = QNetwork(self.state_size, self.action_size)
            
        ## Memory buffer definition
        if not self.using_per :
            self.RAM = NormalBuffer(self.memory_size, self.policy_type)   
        else:
            self.RAM = PrioritizedBuffer(self.memory_size)
        
        self.optimizer = optim.AdamW(self.Pred_model.parameters(), self.learning_rate) # Adam Optimizer but with L2 Regularization implemented
        self.loss_func = nn.MSELoss()
        self.episode_loss = 0.0
        self.running_loss = 0.0
        self.training_loss = []
        self.x_episode = []
        self.counter = 0
        
        if self.run_mode == "test":
            self.load_model = True
        
        # Switches between training on initial weights and weights loaded from the pre-trained episode
        if self.load_model:
            loaded_state_dict = torch.load(self.save_path + str(self.load_episode)+'.pt')
            self.Pred_model.load_state_dict(loaded_state_dict)
    
    # def getQvalue(self, reward, next_target, done):
    #     done = done.numpy()
    #     if done.any():
    #         return (reward)
    #     else:
    #         return (reward + self.discount_factor * next_target.max(1)[0])

    def updateTargetModel(self):
        self.Target_model.load_state_dict(self.Pred_model.state_dict())

    def getAction(self, state):
        state = torch.Tensor(np.array(state)).unsqueeze(0) # get a 1D array
        self.q_value = self.Pred_model(state)
        
        if self.run_mode == "train":
            if np.random.rand() <= self.epsilon:
                self.q_value = np.zeros(self.action_size)
                action = random.randrange(self.action_size)
            else:
                action = int(torch.argmax(self.q_value))
                print("(*) Predicted action: ", action)
            
        if self.run_mode =="test":
            action = int(torch.argmax(self.q_value))
            
        return action
        
    def TrainModel(self):
        ## Check off-policy algorithm
        if self.policy_type == "off":
            states, actions, rewards, next_states, dones = self.RAM.sample(self.batch_size)
            states = torch.Tensor(np.array(states))
            actions = torch.Tensor(actions)
            actions = actions.type(torch.int64).unsqueeze(-1)
            next_states = torch.Tensor(np.array(next_states))
            
            if self.net_type == "normal" or self.net_type == "dueling":
                
                ## get max Q value for state->next_state using q_target_net
                next_q_value = torch.max(self.Target_model(next_states), dim=1)[0]

                ## check if the episode terminates in next step
                q_star = np.zeros(self.batch_size)
                for i in range(self.batch_size):
                    if dones[i]:
                        q_star[i] = rewards[i]
                    else:
                        q_star[i] = rewards[i] + self.discount_factor * next_q_value[i]
                        
                
            elif self.net_type == "double" or self.net_type == "d3q":
                
                ## get Q value for state->next_state using q_target_net
                next_q_value = self.Target_model.forward(next_states)
                
                ## get evaluation_value for the next_state, then compute max_eval_action
                q_eval_value = self.Pred_model.forward(next_states).detach().numpy()

                ## check if the episode terminates in next step
                q_star = np.zeros(self.batch_size)
                for i in range(self.batch_size):
                    if dones[i]:
                        q_star[i] = rewards[i]
                    else:
                        max_eval_actions = np.argmax(q_eval_value[i])
                        q_star[i] = rewards[i] + self.discount_factor * next_q_value[i][int(max_eval_actions)]
                        
        ## Check on-policy algorithm
        if self.policy_type == "on":
            states, actions, rewards, next_states, next_actions, dones = self.RAM.sample(self.batch_size)
            states = torch.Tensor(np.array(states))
            actions = torch.Tensor(actions)
            actions = actions.type(torch.int64).unsqueeze(-1)
            next_states = torch.Tensor(np.array(next_states))
            next_actions = torch.Tensor(next_actions)
            next_actions = next_actions.type(torch.int64).unsqueeze(-1)
            
            if self.net_type == "normal" or self.net_type == "dueling":
                ## get next_q_value from next_actions in next_states
                next_q_value = self.Target_model(next_states).gather(1, next_actions).squeeze()

                ## check if the episode terminates in next step, then compute q_star values
                q_star = np.zeros(self.batch_size)
                for i in range(self.batch_size):
                    if dones[i]:
                        q_star[i] = rewards[i]
                    else:
                        q_star[i] = rewards[i] + self.discount_factor * next_q_value[i]
                        
                        
            elif self.net_type == "double" or self.net_type == "d3q":
                
                ## get Q value for state->next_state using q_target_net
                next_q_value = self.Target_model.forward(next_states).gather(1, next_actions).squeeze()
                
                ## get evaluation_value for the next_state, then compute max_eval_action
                q_eval_value = self.Pred_model.forward(next_states).gather(1, next_actions).squeeze()

                ## check if the episode terminates in next step
                q_star = np.zeros(self.batch_size)
                for i in range(self.batch_size):
                    if dones[i]:
                        q_star[i] = rewards[i]
                    else:
                        next_eval_actions = np.argmax(q_eval_value[i])
                        q_star[i] = rewards[i] + self.discount_factor * next_q_value[i][int(next_eval_actions)]
                        
        
        ## convert td_target to tensor
        td_target = torch.Tensor(q_star)

        ## get predicted_values
        current_q_value = self.Pred_model(states).gather(1, actions).squeeze()

        ## calculate the loss 
        self.loss = self.loss_func(current_q_value, td_target)

        ## perform backprop and update weights
        self.optimizer.zero_grad()
        self.loss.backward()
        self.optimizer.step()
        
        ## analysis loss function
        self.episode_loss += current_q_value.shape[0] * self.loss.item()
        self.running_loss += self.loss.item()
        cal_loss = self.episode_loss / len(states)
        self.training_loss.append(cal_loss)
        self.counter += 1
        self.x_episode.append(self.counter)
        np.savetxt(self.loss_dir + '/loss.csv', self.training_loss, delimiter = ' , ')
        
