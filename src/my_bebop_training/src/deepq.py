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
import torch
from pgmpy.inference import VariableElimination
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
        torch.cuda.empty_cache()


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
        inputs = layers.Input(shape=(3200))
        layer5 = layers.Dense(512, activation="relu")(inputs)
        action = layers.Dense(8, activation="linear")(layer5)
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

        action_probs = self.model([state_tensor], training=False)
        # Take best action
        #action = tf.argmax(action_probs[0]).numpy()


        #predicted = self.model.predict(state)#.reshape(1, len(state)))
        #print("termino predict")
        return action_probs[0].numpy()#action#predicted[0]

    def getTargetQValues(self, state):
        #predicted = self.targetModel.predict(state)#.reshape(1, len(state)))

        state_tensor = tf.convert_to_tensor(state)
        state_tensor = tf.expand_dims(state_tensor, 0)

        action_probs = self.targetModel([state_tensor], training=False)
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
    def selectAction(self, qValues, explorationRate,prob_action,prob_action_n,hay_modelo):
        #--aqui debe de decidir si elegir la accion de la red o no
        #quitamos los que no estan conectados, es decir no aparecen en la red ya que sino esta
        #manda un error al poner la evidenciad ese nodo

        prob_caulsa_model=0.8
        umbral_prob=0.75

        rand = random.random()
        if rand < explorationRate:
            print("accion aleatoria")
            action = np.random.randint(0, self.output_size)
        else:
            print("qvalues")
            action = self.getMaxIndex(qValues)
            if hay_modelo>0:
                rand = random.random()
                if rand < prob_caulsa_model:
                    print("consular al modelo")
                    ##si lleva a una negativa, debe seleccionarse una aleatoria con que no sea esa
                    if prob_action_n[action]>umbral_prob:
                        print("accion selecciona va a negativa! seleccionar otra",prob_action_n[action],prob_action_n,action )
                        action_negative=action
                        while action==action_negative:
                            action=np.random.randint(0, self.output_size)
                    else:
                        #si es una buena probabilidad de recibir mejor recompensa la toma
                        if prob_action[action]>umbral_prob:
                            action=action
                            print("action con buena probabilidad de maximizar")
                        #sino, seleccionar la que tenga mas del modelo causal?
                        else:
                            print("accion con mala probabilidad de mazimizar, mejor seleccionar la que tenga mayor probablidad",prob_action[action],prob_action,action)
                            max_index = prob_action.index(max(prob_action))
                            action=max_index
        
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
            X_batch = np.empty([0, 3200], dtype=np.float64)
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

    def loadWeights(self, path,path1,path2):
        self.model.set_weights(load_model(path).get_weights())
        self.targetModel.set_weights(load_model(path1).get_weights())
        self.memory.load(path2)
