"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import time
import numpy as np
import os
import json
from functools import partial
from lero_control_gym.utils.registration import make


def createDataPoint(results, env_config, con_config, savePath):
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


def get_result(results):
    total_trajectory_loss = 0
    # Plot the experiment.
    for i in range(len(results['info'])):
        # Step the environment and print all returned information.
        obs, reward, done, info, action = results['obs'][i], results['reward'][
            i], results['done'][i], results['info'][i], results['action'][i]
        err = float(info[0]['outside_safezone'])
        total_trajectory_loss += err

    total_trajectory_loss = -total_trajectory_loss
    return total_trajectory_loss


class Task():

    def __init__(self, quadcopter3D, fixed_dist_rbc_pid):
        self.quadcopter = quadcopter3D
        self.controller = fixed_dist_rbc_pid

    def executeTask(self):
        """The main function creating, running, and closing an environment.

        """

        # Create an environment

        STEPS = int(self.quadcopter.Path['maxStepsPerRun'])
        START = time.time()
        savePath = "Experiment_Database"
        # Create controller.
        env_func = partial(make,
                           'quadcopter3D',
                           **self.quadcopter
                           )

        ctrl = make("quad3d_fixed_dist_rbcpid",
                    env_func,
                    **self.controller
                    )

        # Run the experiment.
        results = ctrl.run(iterations=STEPS)
        ctrl.close()

        total_trajectory_loss = get_result(results)

        # datapoint = createDataPoint(results, self.quadcopter, self.controller, savePath=savePath)

        END = time.time()
        # print(str(END - START) + " seconds run time")

        return total_trajectory_loss


# if __name__ == "__main__":
#     main()
