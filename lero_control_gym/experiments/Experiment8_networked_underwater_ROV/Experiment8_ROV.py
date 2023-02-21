"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import time,timeit
# import pybullet as p
from functools import partial
import torch
from lero_control_gym.utils.configuration import ConfigFactory
from lero_control_gym.utils.registration import make



def runExperiment():
    """The main function creating, running, and closing an environment.

    """

    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()
    # Create controller.
    env_func = partial(make,
                       'network_underwater_ROV',
                       **config.ROV)

    exit()
    ctrl = make("quad3d_mpc",
                env_func,
                **config.mpc
                )
    ITERATIONS= 2000
    # Run the experiment.
    results = ctrl.run(n_episodes=ITERATIONS)

    total_trajectory_loss = 0
    # Plot the experiment.
    for i in range(len(results['info'])):
        # Step the environment and print all returned information.
        obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], results['info'][
            i], results['action'][i]
        err = float(info[0]['mse'])
        total_trajectory_loss += err

    print("Total Loss MPC " + str(total_trajectory_loss))


    # Run the experiment.


    return







if __name__ == "__main__":

    runExperiment()