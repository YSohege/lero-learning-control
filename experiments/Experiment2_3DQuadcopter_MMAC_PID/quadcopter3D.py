"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import time
import pybullet as p
from functools import partial

from safe_control_gym.utils.configuration import ConfigFactory
from safe_control_gym.utils.registration import make



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
    env_func = partial(make,
                       'quadrotor3D',
                       **config.quadcopter3D
                       )


    ctrl = make("mmacpid",
                env_func,
                **config.mmac_pid
                )

    ITERATIONS= 2000
    # Run the experiment.
    results = ctrl.run(iterations=ITERATIONS)



    # Plot the experiment.
    for i in range(len(results['obs'])):
        # Step the environment and print all returned information.
        obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], \
                                          results['info'][i], results['action'][i]

        # Print the last action and the information returned at each step.
        print(i, '-th step.')
        print(action, '\n', obs, '\n', reward, '\n', done, '\n', info, '\n')

    ctrl.close()




if __name__ == "__main__":
    main()