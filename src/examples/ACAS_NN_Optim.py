# -*- coding: utf-8 -*-
"""
Created on Sun Dec  3 19:51:29 2017

@author: shantanu
@author: kasim
"""

# Interfaced in GDQNACAS.jl
# ACASNN.jl will be deprecated.

import random
import numpy as np
from keras.models import Sequential
from keras.layers.core import Dense, Activation
from keras.optimizers import RMSprop, Adam
from keras.models import load_model


'''
Function to transition to next state
'''
def MakeMove(state, action):
    # Update vertrate
    if (action == 1):
        state[0][5] = 1.666
    elif(action == 2):
        state[0][5] = 3.333
    elif(action == 3):
        state[0][5] = 4.999
    elif(action == 4):
        state[0][5] = 6.666
    elif(action == 5):
        state[0][5] = 8.333
    elif(action == 6):
        state[0][5] = -1.666
    elif(action == 7):
        state[0][5] = -3.333  
    elif(action == 8):
        state[0][5] = -4.999
    elif(action == 9):
        state[0][5] = -6.666
    elif(action == 10):
        state[0][5] = -8.333
    else:
        state[0][5] = 0

    # Update state
    state[0][0] += state[0][3]
    state[0][1] += state[0][4]
    state[0][2] += state[0][5]

    return state

'''
Function to get a reward for taking an action from a state
'''
def GetReward(new_state, action, moves, array_size):
    # Ownship and intruder are not allowed to occlude
    if abs(new_state[0][0] - new_state[1][0]) < 500 and abs(new_state[0][1] - new_state[1][1]) < 500 and abs(new_state[0][2] - new_state[1][2]) < 500:
        return -1
    # If the planes are more than 500 units away we are good
    elif moves == array_size:
        return +100
    elif (action == 1):
        return -1
    elif(action == 2):
        return -2
    elif(action == 3):
        return -3
    elif(action == 4):
        return -4
    elif(action == 5):
        return -5
    elif(action == 6):
        return -1
    elif(action == 7):
        return -2
    elif(action == 8):
        return -3
    elif(action == 9):
        return -4
    elif(action == 10):
        return -5
    else:
        return 0

'''
Function to train a NN for predicted optimal action
    - epochs                     : # of epochs for training e.g. 500
    - gamma                      : Q-learning discount factor e.g. 0.97
    - epsilon                    : learning factor e.g. 1
    - ownship_start_states       : start state of ownships
    - intruder_state_transitions : array of states for intruders
'''
def TrainModel(epochs, gamma, epsilon, ownship_start_states, intruder_state_transitions):
    # Create a Network using Keras package ------------------------------------
    model = Sequential()
    model.add(Dense(36, init='lecun_uniform', input_shape=(12,)))
    model.add(Activation('relu'))
    
    model.add(Dense(36, init='lecun_uniform'))
    model.add(Activation('relu'))
    
    model.add(Dense(36, init='lecun_uniform'))
    model.add(Activation('relu'))

    
    model.add(Dense(36, init='lecun_uniform'))
    model.add(Activation('relu'))
    
    model.add(Dense(2, init='lecun_uniform'))
    model.add(Activation('relu'))
    
    adam = Adam(lr=1e-4)
    model.compile(loss='mse',optimizer=adam)
    # -------------------------------------------------------------------------



    # Training the Network ----------------------------------------------------
    for index in range(ownship_start_states):
        for i in range(epochs):
            status = 1
            moves = 0
            state = np.zeros((2, 6))
            state = [ ownship_start_states[index], intruder_state_transitions[index][moves] ]
            while(status == 1): # while game is in progress
                # from state S, run q function to get Q values
                qval = model.predict(state.reshape(1,12), batch_size=1)
                if (random.random() < epsilon): # choose random action
                    action = np.random.randint(0,11)
                else: # choose best action 0 := do nothing, 1 := ascend
                    action = (np.argmax(qval))
                    
                new_state = MakeMove(state, action) # take action
                new_state[1] = intruder_state_transitions[index][moves]
                
                reward = GetReward(new_state, action) # compute reward
                
                newQ = model.predict(state, batch_size=1)
                maxQ = np.max(newQ)
                y = np.zeros((1,11))
                y[:] = qval[:]
                if reward == 5 or reward == -5: # terminal state
                    update = reward
                else:
                    update = (reward + (gamma * maxQ))
                y[0][action] = update # target output
                
                model.fit(state.reshape(1,12), y, batch_size=1, nb_epoch=1, verbose=0)
                state = new_state
                
                if reward == 5 or reward == -5:
                    status = 0
                    print(reward)
                moves += 1
            print("Game #: %s completed in: %s moves" % (i,moves))
            if epsilon > 0.1:
                epsilon -= (1/epochs)
                
    model.save('acas_model.h5')
    
'''
Function to predict an optimal action
    - state : current state
    returns : the optimal action
'''
def Predict(state):
    model = load_model('my_model.h5')
    qval = model.predict(state.reshape(1,12), batch_size=1)
    action = (np.argmax(qval))
    
    return action