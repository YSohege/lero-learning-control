"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import random
import time
import numpy as np
from functools import partial
from lero_control_gym.utils.registration import make
import gym
from gym import spaces
class Blended_Control_Task(gym.Env):

    def __init__(self, quadcopter3D , rbc_pid ):
        self.quadcopter = quadcopter3D
        self.controller = rbc_pid

        # Create controller.
        self.env_func = partial(make,
                           'quadcopter3D',
                           **self.quadcopter
                           )

        self.ctrl = make("quad3d_rbcpid",
                    self.env_func,
                    **self.controller
                    )
        self.maxConcentration = 3
        actionS = np.array([self.maxConcentration] *len(self.controller.RBC_DISTRIBUTION_PARAMETERS))
        self.action_space = spaces.MultiDiscrete(actionS)


        #state and target of quadcopter
        self.observation_space = spaces.Box(low=0, high=1, shape=( 1, 4))
        obs, info = self.ctrl.reset()


        self.episode_cum_rew = 0
        self.actions = []
        self.ep_counter = 0
        return




    def reset(self):

        a1 = 0
        a2 = 0
        self.ep_counter+=1
        for action in self.actions:
            a1+=action[0]
            a2+=action[1]

        if len(self.actions)> 0:
            averageA1 = a1/len(self.actions)
            averageA2 = a2/len(self.actions)

        else:
            averageA1 = 0
            averageA2 =0

        print("Ep Done "+ str(self.ep_counter)+": average action = " + str([averageA1, averageA2]) + " , cumulative_ep_reward = " + str(self.episode_cum_rew))
        self.actions = []
        self.episode_cum_rew = 0
        self.quadcopter.Path.randomSeed = random.randint(1,100000)
        self.env_func = partial(make,
                                'quadcopter3D',
                                **self.quadcopter
                                )

        self.ctrl = make("quad3d_rbcpid",
                         self.env_func,
                         **self.controller
                         )

        #initial step with default parameters
        obs, info = self.ctrl.reset()
        info['condensedObs'] = [0,0,0,0]
        obs = info['condensedObs']

        return np.array(obs)

    def step(self, action):
        #offset the action by one - min action is 1
        action = [a+1 for a in action]
        obs, rew, done , info = self.ctrl.step(action)
        # print(str(action) + " " + str(rew) )

        obs = info['condensedObs']
        self.actions.append(action)
        self.episode_cum_rew += rew

        return  np.array(obs) , rew, done , info




    # def executeTask(self):
    #     """The main function creating, running, and closing an environment.
    #
    #     """
    #
    #     # Create an environment
    #
    #     STEPS = int(self.quadcopter.Path['maxStepsPerRun'])
    #     START = time.time()
    #
    #     # Create controller.
    #     env_func = partial(make,
    #                        'quadcopter3D',
    #                        **self.quadcopter
    #                        )
    #
    #
    #     ctrl = make("quad3d_mmacpid",
    #                 env_func,
    #                 **self.controller
    #                 )
    #
    #     # Run the experiment.
    #     results = ctrl.run(iterations=STEPS)
    #
    #     total_trajectory_loss = 0
    #     # Plot the experiment.
    #     for i in range(len(results['info'])):
    #         # Step the environment and print all returned information.
    #         obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], results['info'][
    #             i], results['action'][i]
    #         err = float(info[0]['mse'])
    #         total_trajectory_loss += err
    #
    #     total_trajectory_loss = -total_trajectory_loss
    #
    #     END = time.time()
    #     ctrl.close()
    #     return total_trajectory_loss



# if __name__ == "__main__":
#     main()