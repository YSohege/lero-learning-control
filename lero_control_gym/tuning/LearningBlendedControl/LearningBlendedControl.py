import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import datetime
import os
import gym
import stable_baselines3
from stable_baselines3 import PPO

from functools import partial
from lero_control_gym.utils.registration import make
from lero_control_gym.tasks.Blended_Control_Task.Blended_Control_Task import Blended_Control_Task
from lero_control_gym.utils.configuration import ConfigFactory

# Create an environment





def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()


    env = Blended_Control_Task(**config.Task )



    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=25000)
    model.save("blendingAgent")

    exit()
    return



if __name__ == "__main__":
    main()





        # self.epcount = 0
        # self.action_space = spaces.MultiDiscrete(np.array([self.dirHigh, self.marginHigh]))
        # self.action_space = spaces.Discrete(self.dirHigh)
        #
        # self.low_obs = 0
        # self.high_obs = 1
        # self.input_height = 5  # one candle
        # self.input_width =
        # self.low_state = [[self.low_obs for col in range(self.input_width)] for row in range(self.input_height)]
        # self.high_state = [[self.high_obs for col in range(self.input_width)] for row in range(self.input_height)]
        # self.observation_space = spaces.Box(low=self.low_obs, high=self.high_obs, shape=(1, len(self.low_state)))






def main():

    print(result)

    return



if __name__ == "__main__":
    main()