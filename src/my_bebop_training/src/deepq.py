import random

import numpy as np
from keras import models
from tensorflow.keras import Sequential, optimizers
from tensorflow.keras.layers import Dense, Activation, LeakyReLU, Dropout, Conv2D, MaxPooling2D, Flatten, Dropout, Lambda
from tensorflow.keras.models import load_model
from tensorflow.keras.regularizers import l2
from std_msgs.msg import Int8
import memory
import rospy
from tensorflow.keras import layers
from tensorflow import keras
import tensorflow as tf
import os
import pandas as pd
class DeepQ:
    """
    DQN abstraction.
    As a quick reminder:
        traditional Q-learning:
            Q(s, a) += alpha * (reward(s,a) + gamma * max(Q(s') - Q(s,a))
        DQN:
            target = reward(s,a) + gamma * max(Q(s')
    """

    def __init__(self, inputs, outputs, memorySize, discountFactor, learningRate, learnStart):
        """
        Parameters:
            - inputs: input size
            - outputs: output size
            - memorySize: size of the memory that will store each state
            - discountFactor: the discount factor (gamma)
            - learningRate: learning rate
            - learnStart: steps to happen before for learning. Set to 128
        """
        self.input_size = inputs
        self.output_size = outputs
        self.memory = memory.Memory(memorySize)
        self.discountFactor = discountFactor
        self.learnStart = learnStart
        self.learningRate = learningRate
        self.Ovr = rospy.Subscriber('/turtlebot/save', Int8, self.flag)
        self.save_forward=1
        os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'

        self.red_defined=[[1,1,1,1,1,1,1,0,0],[0,0,0,0,0,0,1,1,0],[0,0,0,0,0,0,1,1,1],[0,0,0,0,0,0,1,2,1],[0,0,0,0,0,0,1,1,2],[0,0,0,0,0,0,1,2,2],[0,0,0,0,0,1,0,2,0], [1,1,1,1,1,1,0,2,0],[0,0,0,0,0,1,0,1,0],[1,1,1,1,1,1,0,1,0],[0,0,0,0,0,1,1,2,0],[1,1,1,1,1,1,1,2,0],[0,0,0,0,0,1,1,1,0],[1,1,1,1,1,1,1,1,0] ]
        self.red_actions=[2,3,0,0,1,1,7,7,6,6,5,5,4,4]#actions
        self.archivo=pd.DataFrame(columns=['dq_0','dq_1','dq_2','dq_3','dq_4','dq_5','dq_6','dq_7'])


    def flag(self,msg):
        self.save_forward = msg.data
    def initNetworks(self, hiddenLayers, training=True, model_path=None):
        print("------------initNetworks----------------------------")
        # Normal: For simulation training
        if training:
            model = self.createModel(hiddenLayers, "relu", self.learningRate)
            self.model = model

            targetModel = self.createModel(hiddenLayers, "relu", self.learningRate)
            self.targetModel = targetModel

        # For simulation testing
        else:
            self.model = models.load_model(model_path)

    def createRegularizedModel(self, hiddenLayers, activationType, learningRate):
        bias = True
        dropout = 0
        regularizationFactor = 0.01
        model = Sequential()
        if len(hiddenLayers) == 0:
            model.add(
                Dense(self.output_size, input_shape=(self.input_size,), kernel_initializer='lecun_uniform', bias=bias))
            model.add(Activation("linear"))
        else:
            if regularizationFactor > 0:
                model.add(Dense(hiddenLayers[0], input_shape=(self.input_size,), kernel_initializer='lecun_uniform',
                                W_regularizer=l2(regularizationFactor), bias=bias))
            else:
                model.add(Dense(hiddenLayers[0], input_shape=(self.input_size,), kernel_initializer='lecun_uniform',
                                bias=bias))

            if activationType == "LeakyReLU":
                model.add(LeakyReLU(alpha=0.01))
            else:
                model.add(Activation(activationType))

            for index in range(1, len(hiddenLayers)):
                layerSize = hiddenLayers[index]
                if regularizationFactor > 0:
                    model.add(
                        Dense(layerSize, kernel_initializer='lecun_uniform', W_regularizer=l2(regularizationFactor),
                              bias=bias))
                else:
                    model.add(Dense(layerSize, kernel_initializer='lecun_uniform', bias=bias))
                if activationType == "LeakyReLU":
                    model.add(LeakyReLU(alpha=0.01))
                else:
                    model.add(Activation(activationType))
                if dropout > 0:
                    model.add(Dropout(dropout))
            model.add(Dense(self.output_size, kernel_initializer='lecun_uniform', bias=bias))
            model.add(Activation("linear"))
        optimizer = optimizers.RMSprop(lr=learningRate, rho=0.9, epsilon=1e-06)
        model.compile(loss="mse", optimizer=optimizer)
        model.summary()
        return model
    def createModel(self, hiddenLayers, activationType, learningRate):
        # Network defined by the Deepmind paper
        inputs = layers.Input(shape=(84, 84, 3,))
        # Convolutions on the frames on the screen
        layer1 = layers.Conv2D(32, 8, strides=4, padding="same",activation="relu")(inputs)
        layer2 = layers.Conv2D(64, 4, strides=2, activation="relu")(layer1)
        layer3 = layers.Conv2D(64, 3, strides=1, activation="relu")(layer2)

        layer4 = layers.Flatten()(layer3)

        layer5 = layers.Dense(512, activation="relu")(layer4)
        action = layers.Dense(self.output_size, activation="linear")(layer5)
        model=keras.Model(inputs=inputs, outputs=action)
        optimizer = optimizers.RMSprop(lr=learningRate, rho=0.9, epsilon=1e-06)
        model.compile(loss="mse", optimizer=optimizer)
        return model

    def printNetwork(self):
        self.model.summary()
        '''
        i = 0
        for layer in self.model.layers:
            weights = layer.get_weights()
            print("layer ", i, ": ", weights)
            i += 1
        '''

    def backupNetwork(self, model, backup):
        weightMatrix = []
        for layer in model.layers:
            weights = layer.get_weights()
            weightMatrix.append(weights)
        i = 0
        for layer in backup.layers:
            weights = weightMatrix[i]
            layer.set_weights(weights)
            i += 1

    def updateTargetNetwork(self):
        self.backupNetwork(self.model, self.targetModel)

    # predict Q values for all the actions
    def getQValues(self, state):
        #print("shape de state",state.shape)
        state_tensor = tf.convert_to_tensor(state)
        state_tensor = tf.expand_dims(state_tensor, 0)
        action_probs = self.model(state_tensor, training=False)
        # Take best action
        #action = tf.argmax(action_probs[0]).numpy()


        #predicted = self.model.predict(state)#.reshape(1, len(state)))
        #print("termino predict")
        return action_probs[0].numpy()#action#predicted[0]

    def getTargetQValues(self, state):
        #predicted = self.targetModel.predict(state)#.reshape(1, len(state)))

        state_tensor = tf.convert_to_tensor(state)
        state_tensor = tf.expand_dims(state_tensor, 0)
        action_probs = self.targetModel(state_tensor, training=False)
        # Take best action
        #action = tf.argmax(action_probs[0]).numpy()

        return action_probs[0].numpy()#action#predicted[0]

    def getMaxQ(self, qValues):
        return np.max(qValues)

    def getMaxIndex(self, qValues):
        return np.argmax(qValues)

    # calculate the target function
    def calculateTarget(self, qValuesNewState, reward, isFinal):
        """
            target = reward(s,a) + gamma * max(Q(s')
        """
        if isFinal:
            return reward
        else:
            return reward + self.discountFactor * self.getMaxQ(qValuesNewState)

    # select the action with the highest Q value
    def selectAction(self, qValues, explorationRate,disc_state):
        #--aqui debe de decidir si elegir la accion de la red o no
        print("space",disc_state)
        new_row= {'dq_0':qValues[0],'dq_1':qValues[1],'dq_2':qValues[2],'dq_3':qValues[3],'dq_4':qValues[4],'dq_5':qValues[5],'dq_6':qValues[6],'dq_7':qValues[7]}
        self.archivo.loc[len(self.archivo)] = new_row
        self.archivo.to_csv('/home/nilda/Documentos/qValues/desde_0.csv', index=False)
        if disc_state in self.red_defined:
            index = self.red_defined.index(disc_state)
            action=self.red_actions[index]
            print(f"The discrete state exists in bayesian at index",index,action)
        else:
            print(f"The discrete state does not exist in bayesian.")
            rand = random.random()
            if rand < explorationRate:
                action = np.random.randint(0, self.output_size)
            else:
                action = self.getMaxIndex(qValues)
            ##--------------------------------------
            '''
            if action==0:
                rospy.logwarn("Action: Forward and Safe: Safe")
                if self.save_forward==0:
                    rospy.logerr("Action: Forward and Safe: NOT Safe")
                    action = np.random.randint(1, self.output_size)
            '''
            ###---------------------------------------
        return action

    def selectActionByProbability(self, qValues, bias):
        qValueSum = 0
        shiftBy = 0
        for value in qValues:
            if value + shiftBy < 0:
                shiftBy = - (value + shiftBy)
        shiftBy += 1e-06

        for value in qValues:
            qValueSum += (value + shiftBy) ** bias

        probabilitySum = 0
        qValueProbabilities = []

        for value in qValues:
            probability = ((value + shiftBy) ** bias) / float(qValueSum)
            qValueProbabilities.append(probability + probabilitySum)
            probabilitySum += probability
        qValueProbabilities[len(qValueProbabilities) - 1] = 1

        rand = random.random()
        i = 0
        for value in qValueProbabilities:
            if rand <= value:
                return i
            i += 1

    def addMemory(self, state, action, reward, newState, isFinal):
        self.memory.addMemory(state, action, reward, newState, isFinal)

    def learnOnLastState(self):
        if self.memory.getCurrentSize() >= 1:
            return self.memory.getMemory(self.memory.getCurrentSize() - 1)

    def learnOnMiniBatch(self, miniBatchSize, useTargetNetwork=True):
        # Do not learn until we've got self.learnStart samples
        if self.memory.getCurrentSize() > self.learnStart:
            # learn in batches of 128
            miniBatch = self.memory.getMiniBatch(miniBatchSize)
            X_batch = np.empty([0, 84,84,3], dtype=np.float64)
            Y_batch = np.empty((0, self.output_size), dtype=np.float64)
            #print(X_batch.shape,Y_batch.shape)
            for sample in miniBatch:
                isFinal = sample['isFinal']
                state = sample['state']
                #print("#------------------- state",state.shape)
                action = sample['action']
                reward = sample['reward']
                newState = sample['newState']

                qValues = self.getQValues(state)
                if useTargetNetwork:
                    qValuesNewState = self.getTargetQValues(newState)
                else:
                    qValuesNewState = self.getQValues(newState)
                targetValue = self.calculateTarget(qValuesNewState, reward, isFinal)

                X_batch = np.append(X_batch, [state.copy()], axis=0)
                Y_sample = qValues.copy()
                Y_sample[action] = targetValue
                Y_batch = np.append(Y_batch, np.array([Y_sample]), axis=0)
                #print(X_batch.shape,Y_batch.shape)
                if isFinal:
                    X_batch = np.append(X_batch, [newState.copy()], axis=0)
                    Y_batch = np.append(Y_batch, np.array([[reward] * self.output_size]), axis=0)
            self.model.fit(X_batch, Y_batch, batch_size=len(miniBatch), epochs=1, verbose=0)

    def saveModel(self, path,path2,path3):
        self.model.save(path)
        self.targetModel.save(path2)
        self.memory.save(path3)

    def loadWeights(self, path):
        self.model.set_weights(load_model(path).get_weights())
