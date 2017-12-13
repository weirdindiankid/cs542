# Filename: ACAS_JLCompat.py
# Author: Dharmesh Tarapore <dharmesh@bu.edu>
# Description: A Julia compatible NNet Python module.
import sys
import os
import numpy as np 
import random
import RLESInterface
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
    model.add(Dense(36, init='lecun_uniform', input_shape=(14,)))
    model.add(Activation('relu'))
    
    model.add(Dense(36, init='lecun_uniform'))
    model.add(Activation('relu'))
    
    model.add(Dense(36, init='lecun_uniform'))
    model.add(Activation('relu'))

    
    model.add(Dense(36, init='lecun_uniform'))
    model.add(Activation('relu'))
    
    model.add(Dense(2, init='lecun_uniform'))
    model.add(Activation('relu'))

    def both(y_true, y_pred):
      d = y_true-y_pred
      a = d**2
      b = 0
      return T.switch(y_true<0,a,b)
    
    adam = Adam(lr=1e-4)
    model.compile(loss=both,optimizer=adam)
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
                qval = model.predict(state.reshape(1,14), batch_size=128)
                if (random.random() < epsilon): # choose random action
                    action = np.random.randint(0,13)
                else: # choose best action 0 := do nothing, 1 := ascend
                    action = (np.argmax(qval))
                    
                new_state = MakeMove(state, action) # take action
                new_state[1] = intruder_state_transitions[index][moves]
                
                reward = GetReward(new_state, action) # compute reward
                
                newQ = model.predict(state, batch_size=128)
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
            print("ACAS iteration #: %s completed in: %s moves" % (i,moves))
            if epsilon > 0.1:
                epsilon -= (1/epochs)
                
    model.save('acas_model.h5')


class ExperienceMemory(object):

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
  		self.discardSample()
  
  def canTrain(self):
  	return self.last_index + 1 - self.first_index >=self.init_size

  def isFull(self):
  	return self.last_index + 1 - self.first_index >= self.capacity

  def isEmpty(self):
  	return self.first_index == -1

  def discardSample(self):
  	rand_index = self.first_index #random.randint(self.first_index, self.last_index)
     del self.memory[rand_index]
     self.first_index += 1

  def obtainSampleForState(self):
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


class Q(object):
    def __init__(self, input_shape, batch_size, num_actions, discount, update_rule, rng=0):
        self.input_shape = input_shape
        self.batch_size = batch_size
        self.num_actions = num_actions
        self.discount = discount
        self.update_rule = update_rule
        self.rng = rng if rng else np.random.RandomState()
        self.actions = ACTIONS
        self.saveFreq        = SAVE_FREQ
        self.experienceStartSize = INIT_SIZE
        self.finalExploration = FINAL_EXPLORATION
        self.finalExplorationSize = FINAL_EXPLORATION_FRAME
        self.targetNetworkUpdateFrequency = TARGET_UPDATE_FREQ
        
        self.initialize_network()
        self.update_counter = 0
        self.counter = -1.0
        self.K_SIZE = K_SIZE
        self.COC = COC

    def obtainNextActionForState(self,state):
        self.counter+=1
        if self.counter < self.experieceStartSize:
            return self.actions[self.rng.randint(self.num_actions)]     
        else:
            num = self.rng.rand()
            actInd=0
            if num>=np.min([(self.counter-self.experienceStartSize),self.finalExplorationSize])/self.finalExplorationSize*(1-self.finalExploration):
                actInd = self.rng.randint(self.num_actions)
            else:
                actInd = np.argmax(self.model.predict(state.reshape(1,state.shape[0]),batch_size=1, verbose = 0))
            return self.actions[actInd]

    def initialize_network(self):
        def both(y_true, y_pred):
          d = y_true-y_pred
          a = d**2
          b = 0
          l = T.switch(y_true<0,a,b)
          cost1 = l#T.sum(l, axis=-1)
          return cost1

          target = Sequential()

          target.add(Dense(128, input_dim=self.input_shape, kernel_initializer='uniform', activation='relu'))
          target.add(Dense(512, kernel_initializer='uniform', activation='relu'))
          target.add(Dense(512, kernel_initializer='uniform', activation='relu'))
          target.add(Dense(128, kernel_initializer='uniform', activation='relu'))
          target.add(Dense(128, kernel_initializer='uniform', activation='relu'))
          target.add(Dense(128, kernel_initializer='uniform', activation='relu'))
          target.add(Dense(self.num_actions, kernel_initializer='uniform'))

          target.compile(loss=both,optimizer=self.update_rule)  


          model = Sequential()
          model.add(Dense(36, init='lecun_uniform', input_shape=(14,)))
          model.add(Activation('relu'))
          
          model.add(Dense(36, init='lecun_uniform'))
          model.add(Activation('relu'))
          
          model.add(Dense(36, init='lecun_uniform'))
          model.add(Activation('relu'))

          
          model.add(Dense(36, init='lecun_uniform'))
          model.add(Activation('relu'))
          
          model.add(Dense(2, init='lecun_uniform'))
          model.add(Activation('relu'))

          def both(y_true, y_pred):
            d = y_true-y_pred
            a = d**2
            b = 0
            return T.switch(y_true<0,a,b)
          
          adam = Adam(lr=1e-4)
          model.compile(loss=both,optimizer=adam)

        
    def train(self,(states,actions,rewards,next_states)):
        
        if np.size(states)==1:
            return
        if self.update_counter % self.saveFreq ==0:
            self.saveModel()
        if self.update_counter % self.targetNetworkUpdateFrequency==0:
            self.reset_target_network()
        self.update_counter+=1
                                             
        modelValues = np.zeros((np.size(actions),self.num_actions)) + 1.0
        q_target = self.target.predict(next_states,batch_size = 512)
        for i in range(len(actions)):
            q_target_temp = np.mean(q_target[i*K_SIZE:(i+1)*K_SIZE],axis=0)
            indTarget = np.argmax(q_target_temp)
            indModel  = int(actions[i]*18.0/np.pi)+2
            if actions[i] == self.COC:
                indModel = 5

            reward = np.mean(rewards[i*K_SIZE:(i+1)*K_SIZE])
            modelValues[i,indModel] = reward+self.discount*q_target_temp[indTarget]
                                             
                                             
        self.model.train_on_batch(states,modelValues)
    
    def reset_target_network(self):
        self.target.set_weights(self.model.get_weights())
        #target_weights = self.target.get_weights()
        #self.target.set_weights(self.model.get_weights())
        #self.model.set_weights(target_weights)
    
    def getModel(self):
        return self.model
    def getTarget(self):
        return self.target
    def saveModel(self):
        self.model.save_weights(("ACAS_NN%d.h5" % self.update_counter),overwrite=True)
        
    def test(self,(states,actions,rewards,next_states)):
        if np.size(states)==1:
            return -1
        q_model  = self.model.predict(states,batch_size=512)
        loss = 0.0
        q_target = self.target.predict(next_states,batch_size = 512)
        for i in range(len(actions)):
            indModel  = int(actions[i]*18.0/np.pi)+2
            if actions[i] == self.COC:
                indModel = 5

            reward = np.mean(rewards[i*K_SIZE:(i+1)*K_SIZE])
            q_target_temp = np.mean(q_target[i*K_SIZE:(i+1)*K_SIZE],axis=0)
            indTarget = np.argmax(q_target_temp)
            loss += (q_model[i,indModel]-reward-self.discount*q_target_temp[indTarget])**2
        return loss/len(q_model)


q = Q(NUM_INPUTS,BATCH_SIZE,NUM_ACTIONS,GAMMA,SOLVER)
repMem = ExperienceMemory()
count = 0

dt = DT
dti = DTI
state = sg.randomStateGenerator();
i = 0

while True:
    for j in range(TRAIN_FREQ):
        i+=1
        action = q.getAction(state)
        nextStates,rewards = sg.getNextState(state,action,dt,dti)
        stateNorm,nextStateNorm = sg.normState(state, nextStates)
        repMem.store((stateNorm,action,rewards,nextStateNorm))
        state = nextStates[0]
        count+=1
        if draw:
            pltAcas.updateState(state,action)
            pltAcas.draw()
            time.sleep(0.3)

        if sg.checkRange(state) or i>100:
            i=0
            state = sg.randomStateGenerator()
    if (count % PRINT_FREQ) ==0 and count>=INIT_SIZE:
        print "Samples: %d, Trainings: %d" % (count,(count-INIT_SIZE)/TRAIN_FREQ),"Loss: %.3e" % q.test(repMem.sample_batch())
        sys.stdout.flush()
    elif (count%10000==0):
        print"Samples: %d" % count
        sys.stdout.flush()
    q.train(repMem.sample_batch())

