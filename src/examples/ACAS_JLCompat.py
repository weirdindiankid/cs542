# Filename: ACAS_JLCompat.py
# Author: Dharmesh Tarapore <dharmesh@bu.edu>
# Description: A Julia compatible NNet Python module.
import sys
import os
import numpy as np 
import random
import matplotlib.pyplot as plt 
from keras.models import Sequential
from keras.layers.core import Dense, Activation
from keras.optimizers import RMSprop, Adam
from keras.models import load_model

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


class ReplayMemory(object):

	def __init__(self, batch_size=BATCH_SIZE, init_size=INIT_SIZE, capacity=DEFAULT_CAPACITY):
		self.memory = {}
		self.batch_size = batch_size
		self.first_index = -1
		self.last_index = -1
		self.capacity = capacity
		self.init_size = init_size

  def store(self, sars_tuple):
  	if self.first_index == -1:
  		self.first_index = 0
  		self.last_index += 1
  		self.memory[self.last_index] = sars_tuple   
  	if (self.last_index + 1 - self.first_index) > self.capacity:
  		self.discard_sample()
  
  def canTrain(self):
  	return self.last_index + 1 - self.first_index >=self.init_size

  def is_full(self):
  	return self.last_index + 1 - self.first_index >= self.capacity

  def is_empty(self):
  	return self.first_index == -1

  def discard_sample(self):
  	rand_index = self.first_index #random.randint(self.first_index, self.last_index)
     del self.memory[rand_index]
     self.first_index += 1

  def sample(self):
      if self.is_empty():
      	raise Exception('Unable to sample from replay memory when empty')
      rand_sample_index = random.randint(self.first_index, self.last_index)
      return self.memory[rand_sample_index]

  def sample_batch(self):
      # must insert data into replay memory before sampling
      if not self.canTrain():
          return (-1,-1,-1,-1)
      if self.is_empty():
          raise Exception('Unable to sample from replay memory when empty')

      # determine shape of states
      states_shape = (self.batch_size,) + np.shape(self.memory.values()[0][0])
      rewards_shape = (self.batch_size*K_SIZE) #+ np.shape(self.memory.values()[0][2])
      nextStates_shape = (self.batch_size*K_SIZE,5)

      states = np.empty(states_shape)
      actions = np.empty((self.batch_size, 1))
      rewards = np.empty(rewards_shape)
      next_states = np.empty(nextStates_shape)

      # sample batch_size times from the memory
      for idx in range(self.batch_size):
          state, action, reward, next_state, = self.sample()
          states[idx] = state
          actions[idx] = action
          rewards[idx*K_SIZE:((idx+1)*K_SIZE)] = reward
          next_states[idx*K_SIZE:((idx+1)*K_SIZE)] = next_state

      return (states,actions,rewards,next_states)

