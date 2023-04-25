#!usr/bin/env python3

import rospy
import time 
from datetime import datetime
# Compile path: ~/deep_rl_gazebo/{nodes|save_model|src|..}/<files>.py
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from src.environment import Env
from nodes.models.drl_agent import *

def Training():
    rospy.init_node('deep_rl_model')
    MAX_EPISODES = rospy.get_param('~max_episodes')
    policy_type = rospy.get_param('~policy_type')
    run_mode = rospy.get_param('~run_mode')
    net_type = rospy.get_param('~net_type')
    using_per = rospy.get_param('~using_per')
    load_episodes = rospy.get_param('~load_episodes')
    model_dir = rospy.get_param('~model_dir')
    save_path = os.path.dirname(os.path.realpath(__file__))
    save_path = save_path.replace('deep_rl_gazebo/nodes', str(model_dir))
    LOG_DATA_DIR = save_path + '/log_data'
    load_episodes = 0
    
    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    result = Float32MultiArray()
    get_action = Float32MultiArray()

    # state_size = 182
    state_size = 22
    action_size = 5
    
    # define the environment
    env = Env(action_size)

    # define the agent
    agent = DeepReinforceAgent(state_size, action_size, policy_type, net_type, using_per, run_mode, load_episodes, str(model_dir))
    episodes = []
    steps_per_episode = []
    total_rewards = []
    global_step = 0
    goal_counters_array = []
    
    # Init log files
    log_sim_info = open(LOG_DATA_DIR + '/LogInfo.txt','w+')
    
    # Date / Time
    start_time = time.time()
    now_start = datetime.now()
    dt_string_start = now_start.strftime("%d/%m/%Y %H:%M:%S")

    # Log date to files
    text = '\r\n' + '****************************************************************\n'
    text = text + 'SIMULATION START ==> ' + dt_string_start + '\r\n'
    text = text + 'INITIAL ROBOT POSITION = ( %.2f , %.2f , %.2f ) \r\n' % (env.init_x, env.init_y, 0.0)
    text = text + '****************************************************************\n'
    print(text)
    log_sim_info.write(text)
        
    for e in range(agent.load_episode + 1, MAX_EPISODES):
        text = '\r\n' + '_____ EPISODE: ' + str(e) + ' _____' + '\r\n'
        text = text + '----------------------------------------------------------------\n'
        print(text)
        done = False
        counters = 0
        state = env.reset()
        reward_per_episode = 0
        
        # Training mode for Off-policy algorithm
        if policy_type == 'off':
            pass
        
        elif policy_type == 'on':
            state = np.float32(state)
            action = agent.getAction(state)
            
        else:
            rospy.loginfo('Please specify the right type of the suitable policies! ( on | off )')
            break

        for step in range(agent.episode_step):
            
            # Training mode for Off-policy algorithm
            if policy_type == 'off':
                state = np.float32(state)
                action = agent.getAction(state)

                next_state, reward, done, counters = env.step(action)
                next_state = np.float32(next_state)
                done = np.bool8(done)

                if reward >= 550.:
                    print('***\n-------- Maximum Reward ----------\n****')
                    for _ in range(3):
                        agent.RAM.add_OffPolicy(state, action, reward, next_state, done)
                else:
                    agent.RAM.add_OffPolicy(state, action, reward, next_state, done)
                # RAM.add(state, action, reward, next_state, done)

                if agent.RAM.len >= agent.train_start:
                    if global_step % 1 == 0:
                        agent.TrainModel()

                reward_per_episode += reward
                state = next_state
                get_action.data = [action, reward_per_episode, reward]
                pub_get_action.publish(get_action)
                
            # Training mode for On-policy algorithm
            elif policy_type == 'on':
                next_state, reward, done, counters = env.step(action)
                next_state = np.float32(next_state)
                done = np.bool8(done)
                next_action = agent.getAction(next_state)

                if reward >= 550.:
                    print('***\n-------- Maximum Reward ----------\n****')
                    for _ in range(3):
                        agent.RAM.add_OnPolicy(state, action, reward, next_state, next_action, done)
                else:
                    agent.RAM.add_OnPolicy(state, action, reward, next_state, next_action, done)
                # RAM.add(state, action, reward, next_state, done)

                if agent.RAM.len >= agent.train_start:
                    if global_step % 1 == 0:
                        agent.TrainModel()

                reward_per_episode += reward
                state = next_state
                action = next_action
                get_action.data = [action, next_action, reward_per_episode, reward]
                pub_get_action.publish(get_action)

            if e % 10 == 0:
                torch.save(agent.Pred_model.state_dict(), agent.dirPath + str(e) + '.pt')

            if step >= 1000:
                print('\n==> Time out! Maxed step per episode\n')
                done = True

            if done:
                result.data = [reward_per_episode, action]
                pub_result.publish(result)
                agent.updateTargetModel()
                total_rewards.append(reward_per_episode)
                steps_per_episode.append(step)
                goal_counters_array.append(counters)
                episodes.append(e)
                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)

                rospy.loginfo('Episode: %d , Reward per episode: %.2f , Memory: %d , Epsilon: %.4f , Time: %d:%02d:%02d',
                                e, reward_per_episode, agent.RAM.len, agent.epsilon, h, m, s)
                text = text + 'Episode: %d , Reward per episode: %.2f , Memory: %d , Epsilon: %.4f , Time: %d:%02d:%02d \r\n'%\
                    (e, reward_per_episode, agent.RAM.len, agent.epsilon, h, m, s)
                text = text + '----------------------------------------------------------------\r\n'
                log_sim_info.write('\r\n'+text)
                
                param_keys = ['epsilon']
                param_values = [agent.epsilon]
                param_dictionary = dict(zip(param_keys, param_values))
                break

            global_step += 1    # step counting when the robot takes action for each iteration
            if global_step % agent.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")
                agent.updateTargetModel()
            

        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay
        
        # Save data to directory
        np.savetxt(LOG_DATA_DIR + '/steps_per_episode.csv', steps_per_episode, delimiter = ' , ')
        np.savetxt(LOG_DATA_DIR + '/reward_per_episode.csv', total_rewards, delimiter = ' , ')
        np.savetxt(LOG_DATA_DIR + '/goal_counters_per_episode.csv', goal_counters_array, delimiter = ' , ')
    
    # Close the log file
    log_sim_info.close()
        
        
if __name__ == '__main__':
    try:
        Training()
            
    except rospy.ROSInterruptException:
        print("<--------- Training completed --------->")
        print('Simulation terminated!')
        pass