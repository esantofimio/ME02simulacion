#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import scipy.io as io
import gym
import tensorflow as tf
import tensorflow.contrib.slim as slim
import numpy as np
#import matplotlib.pyplot as plt
from tensorflow import keras
from ns3gym import ns3env


class DqnAgent(object):
    """docstring for DqnAgent"""
    def __init__(self, inNum, outNum):
        super(DqnAgent, self).__init__()
        self.model = keras.Sequential()
        self.model.add(keras.layers.Dense(inNum, input_shape=(inNum,), activation='relu'))
        self.model.add(keras.layers.Dense(outNum, activation='softmax'))
        self.model.compile(optimizer=tf.train.AdamOptimizer(0.001),
                           loss='categorical_crossentropy',
                           metrics=['accuracy'])

    def get_action(self, state):
        return np.argmax(self.model.predict(state)[0])

    def predict(self, next_state):
        return self.model.predict(next_state)[0]

    def fit(self, state, target, action):
        target_f = self.model.predict(state)
        target_f[0][action] = target 
        self.model.fit(state, target_f, epochs=1, verbose=0)


# Environment initialization
port = 5555
simTime = 10 # seconds
startSim = True
stepTime = 0.05 # seconds
seed = 132
simArgs = {"--simTime": simTime,
           "--testArg": 123,
           "--nodeNum": 5,
           "--distance": 500}
debug = False

env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)
#env = gym.make('ns3-v0')


ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)
s_size = ob_space[0].shape[0]
a_size = ac_space.shape[0]

inputQueues = 1
cwSize = 100

agents = []
for i in range (a_size - 1):
    agents.append(DqnAgent(inputQueues, cwSize))

total_episodes = 10
max_env_steps = 100
env._max_episode_steps = max_env_steps

epsilon = 1.0               # exploration rate
epsilon_min = 0.01
epsilon_decay = 0.999

time_history = []
rew_history = []

for e in range(total_episodes):
    state = env.reset()[0]
    print(state)
    state = np.reshape(state, [1, s_size])
    rewardsum = 0
    for time in range(max_env_steps):
        # Choose action
        actions = []
        if np.random.rand(1) < epsilon:
            for agent in agents:
                actions.append(np.random.randint(cwSize))
        else:
            for agent in agents:
                actions.append(agent.get_action(state[:,0]-state[:,1]))

        # Step
        actions.append(100)
        next_state, reward, done, _ = env.step(actions)
        print("Next State, done:  ", next_state, done)
        next_state = next_state[0]
        if done:
            print("episode: {}/{}, time: {}, rew: {}, eps: {:.2}"
                  .format(e, total_episodes, time, rewardsum, epsilon))
            break

        print("pre resize: ", next_state)
        next_state = np.reshape(next_state, [1, s_size])
        print("post resize: ", next_state)

        
        targets = []

        # Train
        for agent in agents:
            targets.append(reward)

        if not done:
            for i in range(len(agents)):
                targets[i] = reward + 0.95 * np.amax(agents[i].predict(next_state[:,0]-next_state[:,1]))

        for i in range(len(agents)):
            agents[i].fit(state[:,i]-state[:, i + 1], targets[i], actions[i])

        state = next_state
        rewardsum += reward
        if epsilon > epsilon_min: epsilon *= epsilon_decay
        
    time_history.append(time)
    rew_history.append(rewardsum)

#for n in range(2 ** s_size):
#    state = [n >> i & 1 for i in range(0, 2)]
#    state = np.reshape(state, [1, s_size])
#    print("state " + str(state) 
#        + " -> prediction " + str(model.predict(state)[0])
#        )

#print(model.get_config())
#print(model.to_json())
#print(model.get_weights())

# plt.plot(range(len(time_history)), time_history)
# plt.plot(range(len(rew_history)), rew_history)
# plt.xlabel('Episode')
# plt.ylabel('Time')
# plt.show()


curve0 = np.zeros(shape=(101))
curve1 = np.zeros(shape=(101))
curve2 = np.zeros(shape=(101))
curve3 = np.zeros(shape=(101))

for i in range(101):
    state = np.array([i])
    state = np.reshape(state, [1, 1])

    curve0[i] = agent0.get_action(state)
    curve1[i] = agent1.get_action(state)
    curve2[i] = agent2.get_action(state)
    curve3[i] = agent3.get_action(state)
        
