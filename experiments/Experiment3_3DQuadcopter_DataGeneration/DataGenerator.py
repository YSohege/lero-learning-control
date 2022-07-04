"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import json
import time
import numpy as np
import pybullet as p
from functools import partial
import random
from munch import munchify,unmunchify
from safe_control_gym.utils.configuration import ConfigFactory
from safe_control_gym.utils.registration import make
import os


def createDataPoint( results , env_config, con_config):
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

    return datapoint


def main():
    """The main function creating, running, and closing an environment.

    """

    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()

    # Set iterations and episode counter.
    # ITERATIONS = int(config.quadrotor3d.Path['numberRuns']) * int(config.quadrotor3d.Path['numberRuns'])
    #
    #
    # print(config)
    # print(config.quadcopter3D)
    # print(config.mmac_pid)

        # Start a timer.
    START = time.time()

    # Create controller.

    env_conf = config.quadcopter3D
    env_func = partial(make,
                       'quadrotor3D',
                       **config.quadcopter3D
                       )

    maxP= 100000
    maxI= 100000
    maxD= 100000
    data = {}
    # for p in range(maxP):
    #     for i in range(maxP):
    #         for d in range(maxP):
    #             key = "("+ str(p) + ","+str(i)+","+str(d)+")"
    #             print(key)
    #             data[key] = []


    for i in range(int(config.data_generator['iterations'])):
        episode_config = config.mmacpid

        p1 = random.randint(1, maxP)
        i1 = random.randint(1, maxI)
        d1 = random.randint(1, maxD)
        episode_config.ATTITUDE_CONTROLLER_SET = [
                       [
                         [p1 , p1, 1500],
                         [i1,  i1, 1.2],
                         [d1,  d1, 0]
                      ]
                   ]

        ctrl = make("mmacpid",
                    env_func,
                    **episode_config
                    )

        ITERATIONS= 2000
        results = ctrl.run(iterations=ITERATIONS)
        ctrl.close()

        controller = "(" + str(p1) + "," + str(i1) + "," + str(d1) + ")"
        datapoint = createDataPoint( results, env_conf, episode_config)
        data[controller] = datapoint
        print(controller)
        print( str(i) + "/" +str(int(config.data_generator['iterations'])))

    saveFolder = config.data_generator.dbpath

    files = [f for f in os.listdir(saveFolder)]
    dbString = saveFolder +"/Database" +str(len(files))+ ".json"

    with open(dbString, "w+") as f:
        dataBase = data
        json.dump(dataBase, f)





if __name__ == "__main__":
    main()