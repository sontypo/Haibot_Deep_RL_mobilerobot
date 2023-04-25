#!usr/bin/env python3

import random as rd
import numpy as np
from collections import deque

class dataTree():
    write = 0
    def __init__(self, max_size):
        self.max_size = max_size
        self.tree = np.zeros(2*max_size - 1)
        self.data = np.zeros(max_size, dtype=object)
        
    def _propagate(self, idx, change):
        parent = (idx - 1) // 2

        self.tree[parent] += change

        if parent != 0:
            self._propagate(parent, change)

    def _retrieve(self, idx, s):
        left = 2 * idx + 1
        right = left + 1

        if left >= len(self.tree):
            return idx

        if s <= self.tree[left]:
            return self._retrieve(left, s)
        else:
            return self._retrieve(right, s-self.tree[left])

    def total(self):
        return self.tree[0]

    def add(self, p, data):
        idx = self.write + self.max_size - 1

        self.data[self.write] = data
        self.update(idx, p)

        self.write += 1
        if self.write >= self.max_size:
            self.write = 0

    def update(self, idx, p):
        change = p - self.tree[idx]

        self.tree[idx] = p
        self._propagate(idx, change)

    def get(self, s):
        idx = self._retrieve(0, s)
        dataIdx = idx - self.max_size + 1

        return (idx, self.tree[idx], self.data[dataIdx])
    

class NormalBuffer():
    def __init__(self, size, policy_type):
        self.buffer = deque(maxlen=size)
        self.policy_type = policy_type
        self.maxSize = size
        self.len = 0
        
    def sample(self, count):
        batch = []
        count = min(count, self.len)
        batch = rd.sample(self.buffer, count)
        
        if self.policy_type == 'off':
            states_array = np.float32([array[0] for array in batch])
            actions_array = np.float32([array[1] for array in batch])
            rewards_array = np.float32([array[2] for array in batch])
            next_states_array = np.float32([array[3] for array in batch])
            dones = np.bool8([array[4] for array in batch])
            
            return states_array, actions_array, rewards_array, next_states_array, dones
        
        elif self.policy_type == 'on':
            states_array = np.float32([array[0] for array in batch])
            actions_array = np.float32([array[1] for array in batch])
            rewards_array = np.float32([array[2] for array in batch])
            next_states_array = np.float32([array[3] for array in batch])
            next_actions_array = np.float32([array[4] for array in batch])
            dones = np.bool8([array[4] for array in batch])
            
            return states_array, actions_array, rewards_array, next_states_array, next_actions_array, dones
    
    def len(self):
        return self.len
    
    def add_OffPolicy(self, s, a, r, new_s, d):
        transition = (s, a, r, new_s, d)
        self.len += 1 
        if self.len > self.maxSize:
            self.len = self.maxSize
        self.buffer.append(transition)
        
    def add_OnPolicy(self, s, a, r, new_s, new_a, d):
        transition = (s, a, r, new_s, new_a, d)
        self.len += 1 
        if self.len > self.maxSize:
            self.len = self.maxSize
        self.buffer.append(transition)


class PrioritizedBuffer():
    def __init__(self, max_size):
        self.data_tree = dataTree(max_size)
        self.current_length = 0
        self.alpha = 0.6
        self.beta = 0.4

    def push(self, state, action, reward, next_state, done):
        priority = 1.0 if self.current_length is 0 else self.data_tree.tree.max()
        self.current_length = self.current_length + 1
        #priority = td_error ** self.alpha
        experience = (state, action, np.array([reward]), next_state, done)
        self.data_tree.add(priority, experience)

    def sample(self, batch_size):
        batch_idx, batch, IS_weights = [], [], []
        segment = self.data_tree.total() / batch_size
        p_sum = self.data_tree.tree[0]

        for i in range(batch_size):
            a = segment * i
            b = segment * (i + 1)

            s = rd.uniform(a, b)
            idx, p, data = self.data_tree.get(s)

            batch_idx.append(idx)
            batch.append(data)
            prob = p / p_sum
            IS_weight = (self.data_tree.total() * prob) ** (-self.beta)
            IS_weights.append(IS_weight)

        state_batch = []
        action_batch = []
        reward_batch = []
        next_state_batch = []
        done_batch = []

        for transition in batch:
            state, action, reward, next_state, done = transition
            state_batch.append(state)
            action_batch.append(action)
            reward_batch.append(reward)
            next_state_batch.append(next_state)
            done_batch.append(done)

        return state_batch, action_batch, reward_batch, next_state_batch, done_batch, batch_idx, IS_weights

    def update_priority(self, idx, td_error):
        priority = td_error ** self.alpha
        self.data_tree.update(idx, priority)

    def __len__(self):
        return self.current_length