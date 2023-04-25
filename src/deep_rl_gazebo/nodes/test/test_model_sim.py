#!usr/bin/env python3

import os
import time 
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from src.environment import Env
from nodes.models.drl_agent import *
from nodes.common.data_logger import *

def TestModel():
    rospy.init_node('test_sim_node')
    policy_type = rospy.get_param('~policy_type')
    run_mode = rospy.get_param('~run_mode')
    net_type = rospy.get_param('~net_type')
    using_per = rospy.get_param('~using_per')
    load_episodes = rospy.get_param('~load_episodes')
    model_dir = rospy.get_param('~model_dir')
    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    result = Float32MultiArray()
    get_action = Float32MultiArray()
    
    NUMBER_TRIALS = 20
    TRIAL_LENGTH = 5
    state_size = 22
    action_size = 5
    
    # define the environment
    env = Env(action_size)

    # define the agent
    agent = DeepReinforceAgent(state_size, action_size, policy_type, net_type, using_per, run_mode, load_episodes, str(model_dir))
    rewards_per_trial, episodes, reward_list = [], [], []
    global_steps = 0
    
    # Date / Time
    start_time = time.time()
    now_start = datetime.now()
    dt_string_start = now_start.strftime("%d/%m/%Y %H:%M:%S")

    # Log date to files
    text = '\r\n' + '********************************************************\n'
    text = text + 'DEPLOYMENT START ==> ' + dt_string_start + '\r\n'
    text = text + '********************************************************\n'
    print(text)
    
    for e in range(1, NUMBER_TRIALS):
        text = '\r\n' + '_____ TRIAL: ' + str(e) + ' _____' + '\r\n'
        text = text + '-----------------------------------------------------------\n'
        print(text)
        done = False
        state = env.reset()
        score = 0
        
        if policy_type == 'off':
            pass
            
        elif policy_type == 'on':
            state = np.float32(state)
            # get action
            action = agent.getAction(state)
        
        else:
            rospy.loginfo('Please specify the right type of the suitable policies! ( on | off )')
            break
        
        episode_is_done = False
        while not episode_is_done:
            if policy_type == 'off':
                state = np.float32(state)
                # get action
                action = agent.getAction(state)
                
                # take action and return next_state, reward and other status
                next_state, reward, done, counters = env.step(action)
                next_state = np.float32(next_state)
                
                score += reward
                reward_list.append(reward)
                
                # update state, publish actions
                state = next_state
                get_action.data = [action, score, reward]
                pub_get_action.publish(get_action)
            
            elif policy_type == 'on':
                # take action and return next_state, reward and other status
                next_state, reward, done, counters = env.step(action)
                next_state = np.float32(next_state)
                
                # get action
                next_action = agent.getAction(next_state)
                
                score += reward
                reward_list.append(reward)
                
                # update state, publish actions
                state = next_state
                action = next_action
                get_action.data = [action, next_action, score, reward]
                pub_get_action.publish(get_action)
            
            # check if goal is reached
            if env.get_goalbox:
                env.pub_cmd_vel.publish(Twist())
                text = '\r\n'+'Trial: %d | Goal [%d] completed \r\n'%(e, counters)
                print(text)
                
                env.get_goalbox = False
                if counters >= TRIAL_LENGTH:
                    print("Deployment Terminated Successfully!!!")
                    text = '\r\n'+'Trial: %d | Deployment Terminated Successfully!!! \r\n'%(e)
                    text = text + '-----------------------------------------------------------\r\n'
                    print(text)
                    
                    episode_is_done = True
                    # logger.save_data(e)
            
            # check if collision or terminated status      
            if done:
                # logger.save_data(e, done="failed")
                env.pub_cmd_vel.publish(Twist())
                result.data = [score, action]
                pub_result.publish(result)
                agent.updateTargetModel()
                rewards_per_trial.append(score)
                episodes.append(e)
                
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)

                text = '\r\n'+'Failed, at Trial %d | Time: %d:%02d:%02d \r\n'%(e, h, m, s)
                text = text + '-----------------------------------------------------------\r\n'
                print(text)
                
                param_keys = ['epsilon']
                param_values = [agent.epsilon]
                param_dictionary = dict(zip(param_keys, param_values))
                break    
            
            global_steps += 1
            
            

if __name__ == '__main__':
    try:
        TestModel()
    
    except rospy.ROSInterruptException:
        print("<--------- Test mode completed --------->")
        print('Deployment Break!')
        pass