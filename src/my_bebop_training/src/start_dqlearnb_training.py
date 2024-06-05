#!/usr/bin/env python

'''
Based on:
https://github.com/vmayoral/basic_reinforcement_learning
https://gist.github.com/wingedsheep/4199594b02138dd427c22a540d6d6b8d
'''
import gym
from gym import wrappers
import time
import json
import liveplot
import deepq
import rospy
import numpy
import rospkg
import random
import os
from openai_ros_b.openai_ros_common import StartOpenAI_ROS_Environment

import utils
from distutils.dir_util import copy_tree
import numpy as np
import pandas as pd
from pgmpy.models import BayesianModel
from pgmpy.estimators import HillClimbSearch, BicScore, BayesianEstimator
import networkx as nx
from pgmpy.inference import VariableElimination
from itertools import permutations

def detect_monitor_files(training_dir):
    return [os.path.join(training_dir, f) for f in os.listdir(training_dir) if f.startswith('openaigym')]


def clear_monitor_files(training_dir):
    files = detect_monitor_files(training_dir)
    if len(files) == 0:
        return
    for file in files:
        print(file)
        os.unlink(file)


if __name__ == '__main__':
    print("----------------------------name--------------------------")

    # Set Gazebo and ROS Master to different ports for running multiple instances

    ##-------
    ##-------
    model_actions=[]
    graphs=[0,0,0,0,0,0,0,0]
    best_model=[0,0,0,0,0,0,0,0]
    adj_matrixs=[0,0,0,0,0,0,0,0]
    nodes_with_outgoing_edges=[0,0,0,0,0,0,0,0]
    nodes_with_incoming_edges=[0,0,0,0,0,0,0,0]
    depend_reward=[0,0,0,0,0,0,0,0]
    unconnected_nodes=[0,0,0,0,0,0,0,0]
    # Node of interest
    node_of_interest = 'reward'
    models=[0,0,0,0,0,0,0,0]
    inference=[0,0,0,0,0,0,0,0]
    time_t=['section_0','section_1','section_2','section_3','section_4','rearch_goal','angle_goal','altitude','distance_goal']


    random_number = random.randint(10000, 15000)
    port_gazebo = random_number + 1  # os.environ["ROS_PORT_SIM"]
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(port_gazebo)

    rospy.init_node('example_bebop_dqlearn',
                    anonymous=True, log_level=rospy.WARN)

    archivo=pd.DataFrame(columns=['episode','section_0','section_1','section_2','section_3','section_4','see_goal','distance_goal','angle_goal','altitude','section_0_n','section_1_n','section_2_n','section_3_n','section_4_n','see_goal_n','distance_goal_n','angle_goal_n','altitude_n',"action","reward_acumulated","reward"])
    archivo_discreto = pd.DataFrame(columns=['episode','section_0','section_1','section_2','section_3','section_4','see_goal','distance_goal','angle_goal','altitude','section_0_n','section_1_n','section_2_n','section_3_n','section_4_n','see_goal_n','distance_goal_n','angle_goal_n','altitude_n',"action","reward_acumulated","reward"])

    # Init openai_ros_b ENV
    task_and_robot_environment_name = rospy.get_param(
        '/bebop/task_and_robot_environment_name')
    max_ep_steps = rospy.get_param("/bebop/nsteps")
    env = StartOpenAI_ROS_Environment(
        task_and_robot_environment_name, max_ep_steps)

    # Create the Gym environment
    rospy.loginfo("Gym environment done")
    rospy.loginfo("Starting Learning")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('bebop_openai_ros_example')
    outdir = pkg_path + '/training_results/dqlearn'
    path = pkg_path + '/training_results/dqlearn/bebop_'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.loginfo("Monitor Wrapper started")

    # Remove log file if exist
    gazebo_world_launch_name = env.get_gazebo_world_launch_name()
    utils.remove_logfile_if_exist(outdir, gazebo_world_launch_name)

    continue_execution = False
    # fill this if continue_execution=True
    resume_epoch = 'prueba'  # change to epoch to continue from
    resume_path = path + resume_epoch
    weights_path = resume_path + '.h5'
    weights_target_path = resume_path +'_target_' + '.h5'
    memory_path = resume_path +'_memory_' + '.pkl'
    monitor_path = outdir  # resume_path
    params_json = resume_path + '.json'
    print("-------------before continue execution--------------")
    if not continue_execution:
        # Each time we take a sample and update our weights it is called a mini-batch.
        # Each time we run through the entire dataset, it's called an epoch.
        # PARAMETER LIST
        epochs = rospy.get_param("/bebop/nepisodes")  # 1000
        steps = rospy.get_param("/bebop/nsteps")  # 1000
        updateTargetNetwork = 4#10000
        explorationRate = rospy.get_param("/bebop/epsilon")  # 1
        minibatch_size = 32
        learnStart = 64
        learningRate = rospy.get_param("/bebop/alpha")  # 0.00025
        discountFactor = rospy.get_param("/bebop/gamma")  # 0.99
        memorySize = 1000000
        network_inputs = (84,84,3)#7056  # 100
        network_outputs = 8 #3  # 21
        network_structure = [300, 300]
        current_epoch = 0
        last100ScoresIndex = 0
        stepCounter = 0
        last100Filled = False
        highest_reward = 0

        epsilon_discount = rospy.get_param("/bebop/epsilon_discount")  # 0.995

        deepQ = deepq.DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
        deepQ.initNetworks(network_structure)
    else:
        # Load weights, monitor info and parameter info.
        # ADD TRY CATCH fro this else
        with open(params_json) as outfile:
            d = json.load(outfile)
            epochs = d.get('epochs') + 500
            steps = d.get('steps')
            updateTargetNetwork = d.get('updateTargetNetwork')
            explorationRate = d.get('explorationRate')
            minibatch_size = d.get('minibatch_size')
            learnStart = d.get('learnStart')
            learningRate = d.get('learningRate')
            discountFactor = d.get('discountFactor')
            memorySize = d.get('memorySize')
            network_inputs = (84,84,3)#7056  # 100
            network_outputs = 8 #d.get('network_outputs')
            network_structure = d.get('network_structure')
            current_epoch = d.get('current_epoch')
            last100ScoresIndex = d.get('last100ScoresIndex')
            stepCounter =  d.get('stepCounter')
            last100Filled = d.get('last100Filled')
            highest_reward = d.get('highest_reward')

            # added
            epsilon_discount = rospy.get_param("/bebop/epsilon_discount")  # 0.995

        deepQ = deepq.DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
        deepQ.initNetworks(network_structure, training=True)

        deepQ.loadWeights(weights_path,weights_target_path,memory_path)

        clear_monitor_files(outdir)
        copy_tree(monitor_path, outdir)

    env._max_episode_steps = steps  # env returns done after _max_episode_steps
    env = gym.wrappers.Monitor(env, outdir, force=not continue_execution, resume=continue_execution)

    last100Scores = [0] * 100
    rospy.logwarn("Number of episodes:"+str(epochs)+" Steps: "+str(steps))
    start_time = time.time()
    print("Number of episodes",epochs)
    print("\n")
    #rospy.sleep(100)
    #print("finalizo")
    # start iterating from 'current epoch'.
    deepQ.printNetwork()

    hay_modelo=0
    update_networkb=100
    prob_action=[0,0,0,0,0,0,0,0]
    prob_action_n=[0,0,0,0,0,0,0,0]

    #print("-----------before epoch----------------")
    print("updateTargetNetwork",updateTargetNetwork)
    for epoch in range(current_epoch + 1, epochs + 1, 1):
        print("Episode Number:",epoch)
        print("\n")
        #if last100Filled==False:
        #print("before reset",epoch)
        observation = env.reset(num_episode=epoch)
        _observation = numpy.array(observation)
        #print("observation")
        #print("_observation",_observation.shape)
        done = False
        cumulated_reward = 0
        episode_step = 0
        #rospy.sleep(100)
        #print("finalizo")
        # run until env returns done
        while not done:
            #print("while")
            rospy.logwarn("Episode Steps: " + str(episode_step))
            # env.render()
            _observation = numpy.array(_observation)
            #print("before qvalues")
            state_discre=env.return_state_discrete()
            qValues = deepQ.getQValues(_observation)
            #print("before action")
            
            print("discrete desde start training",)
            action = deepQ.selectAction(qValues, explorationRate,prob_action,prob_action_n,hay_modelo)
            #print("action",action)
            newObservation,reward, done, info = env.step(action)
            #print("after newObservation")
            success_episode, failure_episode = env.get_episode_status()
            #print("after newobservation succes")
            print("cumulated reward",cumulated_reward,"reward",reward)
            #rospy.sleep(100)
            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            deepQ.addMemory(_observation, action, reward, newObservation, done)
            #print("after addmemory",done)
            if stepCounter >= learnStart:
                if stepCounter <= updateTargetNetwork:
                    deepQ.learnOnMiniBatch(minibatch_size, False)
                else:
                    deepQ.learnOnMiniBatch(minibatch_size, True)

            _observation = newObservation

            reward_d = 0 if reward < 0 else 1
            reward_ad = 0 if cumulated_reward < 0 else 1

            new_row = {'episode':epoch,'section_0': previous_state[0],'section_1': previous_state[1],'section_2': previous_state[2],'section_3': previous_state[3],'section_4': previous_state[4], 'see_goal': previous_state[5],'distance_goal':previous_state[6],'angle_goal':previous_state[7],'altitude':previous_state[8],'section_0_n':array_states [0],'section_1_n':array_states [1],'section_2_n':array_states [2],'section_3_n':array_states [3],'section_4_n':array_states [4],'see_goal_n':array_states [5],'distance_goal_n':array_states [6],'angle_goal_n':array_states [7],'altitude_n':array_states [8],"action": action, "reward_acumulated": cumulated_reward,"reward": reward}
            new_row_d = {'episode':epoch,'section_0': previous_dstate[0],'section_1': previous_dstate[1],'section_2': previous_dstate[2],'section_3': previous_dstate[3],'section_4': previous_dstate[4], 'see_goal': previous_dstate[5],'distance_goal':previous_dstate[6],'angle_goal':previous_dstate[7],'altitude':previous_dstate[8],'section_0_n':array_states_discrete [0],'section_1_n':array_states_discrete [1],'section_2_n':array_states_discrete [2],'section_3_n':array_states_discrete [3],'section_4_n':array_states_discrete [4],'see_goal_n':array_states_discrete [5],'distance_goal_n':array_states_discrete [6],'angle_goal_n':array_states_discrete [7],'altitude_n':array_states_discrete [8],"action": action, "reward_acumulated": reward_ad, "reward":reward_d}

            archivo.loc[len(archivo)] = new_row
            archivo_discreto.loc[len(archivo_discreto)] = new_row_d

            print("len",len(archivo_discreto))
            if len(archivo_discreto)% update_networkb == 0:
                print("----------actualizar red bayesiana---------------")
                model_actions=[]
                for i in range(8):
                    model_actions.append(archivo_discreto[archivo_discreto['action'] == i])
                    # model_actions es solo el archivo csv
                    ##--- esto es una vez que ya se ha dividido por acciones
                    print(len(model_actions[i]))
                    if len(model_actions[i])>0:
                        bic_score = BicScore(model_actions[i])
                        if (len(archivo_discreto)==update_networkb):
                            hcs = HillClimbSearch(model_actions[i])
                        else:
                            if (graphs[i]!=0):
                                initial_model = BayesianModel(graphs[i].edges())
                                hcs = HillClimbSearch(model_actions[i],initial_model)
                            else:
                                hcs = HillClimbSearch(model_actions[i])
                        best_model[i] = hcs.estimate(scoring_method=bic_score)
                        print(best_model[i].edges())
                        if (len(best_model[i].edges())>0):
                            graphs[i] = nx.DiGraph()
                            graphs[i].add_edges_from(best_model[i].edges())  # Add edges from the Bayesian model

                            ##---- quit the sincrono edges
                            for (u, v) in permutations(set(time_t), 2):  # Use permutations to consider all directional pairs
                                if graphs[i].has_edge(u, v):
                                    graphs[i].remove_edge(u, v)
                            # Save the DataFrame to a CSV file for external viewing
                            adj_matrix_sp = nx.adjacency_matrix(graphs[i]) 
                            adj_matrixs[i] = pd.DataFrame(adj_matrix_sp.todense(),
                                                        index=graphs[i].nodes(), columns=graphs[i].nodes())
                            print(adj_matrixs[i])
                            adj_matrixs[i].to_csv('/home/nilda/Documentos/AdjMatrix/action_'+str(i)+'.csv')

                            models[i] = BayesianModel(best_model[i].edges())
                            models[i].fit(model_actions[i], estimator=BayesianEstimator, prior_type="BDeu") ##--- aqui ya se tienen las cpt

                            all_nodes = set(model_actions[i].columns)  # Assuming each column represents a node
                            edges = best_model[i].edges()
                            # Find all unique nodes with edges
                            nodes_with_outgoing_edges[i] = {edge[0] for edge in edges}  # Nodes with outgoing edges
                            nodes_with_incoming_edges[i] = {edge[1] for edge in edges}  # Nodes with incoming edges
                            # Get parents (predecessors)
                            if graphs[i].has_node(node_of_interest):
                                parents = list(graphs[i].predecessors(node_of_interest))
                                #print(f"Parents of {node_of_interest}:", parents)
                                # Get children (successors)
                                children = list(graphs[i].successors(node_of_interest))
                                #print(f"Children of {node_of_interest}:", children)
                                #padres de padres
                                parents_of_parents = set()
                                for parent in parents:
                                    parents_of_parents.update(get_unique_parents(graphs[i], parent))
                                # Get parents of each child
                                parents_of_children = set()
                                for child in children:
                                    parents_of_children.update(get_unique_parents(graphs[i], child))
                                # Convert lists to sets and take the union
                                depend_reward[i] = list(set(parents) | set(children)| parents_of_children | parents_of_parents)
                                connected_nodes = nodes_with_outgoing_edges[i].union(nodes_with_incoming_edges[i])
                                unconnected_nodes[i] = all_nodes - connected_nodes
                            else:
                                depend_reward[i]=[]
                            inference[i] = VariableElimination(models[i])
                hay_modelo=1
                evidence = {'section_0':previous_dstate[0],'section_1':previous_dstate[1],
                    'section_2':previous_dstate[2],'section_3':previous_dstate[3],
                    'section_4':previous_dstate[4], 'rearch_goal':previous_dstate[5],
                    'distance_goal':previous_dstate[6],'angle_goal':previous_dstate[7],
                    'altitude': previous_dstate[8],
                    }
                
                for i in range(8): ##para todas las acciones
                    if unconnected_nodes[i]==0:
                        prob_action[i]=0
                    else:
                        try:
                            evidence = {key: value for key, value in evidence.items() if key not in unconnected_nodes[i]} #preguntarse que hacer cuando es vacio
                            filtered_evidence= {key: value for key, value in evidence.items() if key in depend_reward[i]}
                            result = inference[i].query(variables=['reward'], evidence=filtered_evidence)
                            for state in result.state_names['reward']:
                                #print(f"reward = {state}: {result.values[result.state_names['reward'].index(state)]}")
                                if (state==1):
                                    prob_action[i]=result.values[result.state_names['reward'].index(state)]
                                else:
                                    prob_action_n[i]=result.values[result.state_names['reward'].index(0)]
                        except:
                                prob_action[i]=0
                                prob_action_n[i]=0
                print("probs",prob_action,prob_action_n)
                

            if done:
                data = [epoch, success_episode, failure_episode, cumulated_reward, episode_step + 1]
                utils.record_data(data, outdir, gazebo_world_launch_name)
                print("EPISODE REWARD: ", cumulated_reward)
                print("EPISODE STEP: ", episode_step + 1)
                print("EPISODE SUCCESS: ", success_episode)
                print("EPISODE FAILURE: ", failure_episode)
                last100Scores[last100ScoresIndex] = episode_step
                last100ScoresIndex += 1
                if last100ScoresIndex >= 10:
                    last100Filled = True
                    last100ScoresIndex = 0
                if not last100Filled:
                    print("EP " + str(epoch) + " - " + format(episode_step + 1) + "/" + str(
                        steps) + " Episode steps   Exploration=" + str(round(explorationRate, 2)))
                else:
                    m, s = divmod(int(time.time() - start_time), 60)
                    h, m = divmod(m, 60)
                    print("EP " + str(epoch) + " - " + format(episode_step + 1) + "/" + str(
                        steps) + " Episode steps - last100 Steps : " + str(
                        (sum(last100Scores) / len(last100Scores))) + " - Cumulated R: " + str(
                        cumulated_reward) + "   Eps=" + str(round(explorationRate, 2)) + "     Time: %d:%02d:%02d" % (
                          h, m, s))
                    if epoch % 10 == 0:
                        # save model weights and monitoring data every 100 epochs.
                        deepQ.saveModel(path + str(epoch) + '.h5',path + str(epoch)+'_target_' + '.h5',path + str(epoch)+'_memory_' + '.pkl')
                        env._flush()
                        copy_tree(outdir, str(epoch))
                        # save simulation parameters.
                        parameter_keys = ['epochs', 'steps', 'updateTargetNetwork', 'explorationRate', 'minibatch_size',
                                            'learnStart', 'learningRate', 'discountFactor', 'memorySize',
                                            'network_inputs', 'network_outputs', 'network_structure', 'current_epoch','stepCounter','last100ScoresIndex',
                                            'last100Filled','highest_reward']
                        parameter_values = [epochs, steps, updateTargetNetwork, explorationRate, minibatch_size,
                                            learnStart, learningRate, discountFactor, memorySize, network_inputs,
                                            network_outputs, network_structure, epoch, stepCounter,last100ScoresIndex,
                                            last100Filled,highest_reward]
                        parameter_dictionary = dict(zip(parameter_keys, parameter_values))
                        with open(path + str(epoch) + '.json', 'w') as outfile:
                            json.dump(parameter_dictionary, outfile)
                        archivo.to_csv('/home/nilda/Documentos/Resultados/rewards.csv', index=False)
                        archivo_discreto.to_csv('/home/nilda/Documentos/Resultados/rewards_discreto.csv', index=False)

            stepCounter += 1
            if stepCounter % updateTargetNetwork == 0:
                deepQ.updateTargetNetwork()
                print("updating target network")

            episode_step += 1

        explorationRate *= epsilon_discount
        explorationRate = max(0.05, explorationRate)

    print("----------------------------training end------------------------------------")
    env.close()
