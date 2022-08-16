"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import random
import time
import numpy as np
import json
import os
from functools import partial
from lero_control_gym.utils.registration import make
import gym
from gym import spaces
class Task(gym.Env):

    def __init__(self, quadcopter3D , rl_rbc_pid ):
        self.quadcopter = quadcopter3D
        self.controller = rl_rbc_pid


        return




    # def reset(self):
    #
    #     a1 = 0
    #     a2 = 0
    #
    #     for action in self.actions:
    #         a1+=action[0]
    #         a2+=action[1]
    #
    #     if len(self.actions)> 0:
    #         averageA1 = a1/len(self.actions)
    #         averageA2 = a2/len(self.actions)
    #
    #     else:
    #         averageA1 = 0
    #         averageA2 =0
    #
    #     print("Ep Done: average action = " + str([averageA1, averageA2]) + " , cumulative_ep_reward = " + str(self.episode_cum_rew))
    #     self.actions = []
    #     self.episode_cum_rew = 0
    #
    #     self.env_func = partial(make,
    #                             'quadcopter3D',
    #                             **self.quadcopter
    #                             )
    #
    #     self.ctrl = make("quad3d_rbcpid",
    #                      self.env_func,
    #                      **self.controller
    #                      )
    #
    #     #initial step with default parameters
    #     obs, info = self.ctrl.reset()
    #
    #     return np.array(obs)
    #
    # def step(self, action):
    #     #offset the action by one - min action is 1
    #     action = [a+1 for a in action]
    #     obs, rew, done , info = self.ctrl.step(action)
    #     # print(str(action) + " " + str(rew) )
    #
    #     self.actions.append(action)
    #
    #
    #     self.episode_cum_rew += rew
    #     return  np.array(obs) , rew, done , info
    #
    #

    def createDataPoint(self, results, env_config, con_config, savePath):
        datapoint = {
            "obs": [],
            "info": [],
            "action": [],
            "reward": [],
            "done": [],
            "env_config": env_config.Env,
            "control_config": con_config

        }
        for point in range(len(results.obs)):
            datapoint['obs'].append(np.array(results.obs[point]).tolist())
            datapoint['info'].append(np.array(results.info[point]).tolist())
            datapoint['action'].append(np.array(results.action[point]).tolist())
            datapoint['reward'].append(np.array(results.reward[point]).tolist())

        saveFolder = savePath
        files = [f for f in os.listdir(saveFolder)]
        dbString = saveFolder + "/results-run-" + str(len(files)) + ".json"

        with open(dbString, "w+") as f:
            dataBase = datapoint
            json.dump(dataBase, f)

        return datapoint

    def get_result(self, results):
        total_trajectory_loss = 0
        # Plot the experiment.
        for i in range(len(results['info'])):
            # Step the environment and print all returned information.
            obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], \
                                              results['info'][
                                                  i], results['action'][i]
            err = float(info[0]['outside_safezone'])
            total_trajectory_loss += err

        total_trajectory_loss = -total_trajectory_loss
        return total_trajectory_loss

    def get_average_action(self, results):

        av_action_P = []
        av_action_D = []
        # Plot the experiment.
        for i in range(len(results['info'])):
            # Step the environment and print all returned information.
            obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], \
                                              results['info'][i], results['action'][i]
            av_action_P.append(float(action[0]))
            av_action_D.append(float(action[1]))

        action_P = np.average(av_action_P)
        action_D = np.average(av_action_D)

        return [action_P, action_D]


    def executeTask(self):
        """The main function creating, running, and closing an environment.

        """

        # Create an environment

        STEPS = int(self.quadcopter.Path['maxStepsPerRun'])
        START = time.time()

        # Create controller.
        env_func = partial(make,
                           'quadcopter3D',
                           **self.quadcopter
                           )


        ctrl = make("quad3d_rl_rbcpid",
                    env_func,
                    **self.controller
                    )

        # Run the experiment.
        results = ctrl.run(iterations=STEPS)

        total_trajectory_loss = self.get_result(results)
        average_action = self.get_average_action(results)
        savePath = "Experiment_Database"
        # datapoint = self.createDataPoint(results, self.quadcopter, self.controller, savePath=savePath)

        END = time.time()
        # print(str(END - START) + " seconds run time")

        return total_trajectory_loss , average_action



# if __name__ == "__main__":
#     main()