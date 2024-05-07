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
    random_number = random.randint(10000, 15000)
    port_gazebo = random_number + 1  # os.environ["ROS_PORT_SIM"]
    os.environ["GAZEBO_MASTER_URI"] = "http://localhost:" + str(port_gazebo)

    rospy.init_node('example_bebop_dqlearn',
                    anonymous=True, log_level=rospy.WARN)

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
    plotter = liveplot.LivePlot(outdir)
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

            # added
            epsilon_discount = rospy.get_param("/bebop/epsilon_discount")  # 0.995

        deepQ = deepq.DeepQ(network_inputs, network_outputs, memorySize, discountFactor, learningRate, learnStart)
        deepQ.initNetworks(network_structure, training=True)

        deepQ.loadWeights(weights_path)

        clear_monitor_files(outdir)
        copy_tree(monitor_path, outdir)

    env._max_episode_steps = steps  # env returns done after _max_episode_steps
    env = gym.wrappers.Monitor(env, outdir, force=not continue_execution, resume=continue_execution)

    last100Scores = [0] * 100
    last100ScoresIndex = 0
    last100Filled = False
    stepCounter = 0
    highest_reward = 0
    rospy.logwarn("Number of episodes:"+str(epochs)+" Steps: "+str(steps))
    start_time = time.time()
    print("Number of episodes",epochs)
    print("\n")
    #rospy.sleep(100)
    #print("finalizo")
    # start iterating from 'current epoch'.
    deepQ.printNetwork()
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
            action = deepQ.selectAction(qValues, explorationRate,state_discre)
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
                        deepQ.saveModel(path + str(epoch) + '.h5')
                        env._flush()
                        copy_tree(outdir, str(epoch))
                        # save simulation parameters.
                        parameter_keys = ['epochs', 'steps', 'updateTargetNetwork', 'explorationRate', 'minibatch_size',
                                            'learnStart', 'learningRate', 'discountFactor', 'memorySize',
                                            'network_inputs', 'network_outputs', 'network_structure', 'current_epoch']
                        parameter_values = [epochs, steps, updateTargetNetwork, explorationRate, minibatch_size,
                                            learnStart, learningRate, discountFactor, memorySize, network_inputs,
                                            network_outputs, network_structure, epoch]
                        parameter_dictionary = dict(zip(parameter_keys, parameter_values))
                        with open(path + str(epoch) + '.json', 'w') as outfile:
                            json.dump(parameter_dictionary, outfile)

            stepCounter += 1
            if stepCounter % updateTargetNetwork == 0:
                deepQ.updateTargetNetwork()
                print("updating target network")

            episode_step += 1

        explorationRate *= epsilon_discount
        explorationRate = max(0.05, explorationRate)

        if epoch % 100 == 0:
            plotter.plot(env)
    print("----------------------------training end------------------------------------")
    env.close()
