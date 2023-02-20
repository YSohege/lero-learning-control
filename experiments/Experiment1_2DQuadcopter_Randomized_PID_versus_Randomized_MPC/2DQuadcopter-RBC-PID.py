"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 ./pid_experiment.py --task quadrotor --algo pid --overrides ./config_pid_quadrotor.yaml

"""
import time
import pybullet as p
from functools import partial

from lero_control_gym.utils.configuration import ConfigFactory
from lero_control_gym.utils.registration import make
from lero_control_gym.utils.registration import make

def main():
    """The main function creating, running, and closing an environment.

    """

    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()

    # Set iterations and episode counter.
    ITERATIONS = int(config.quadrotor_config['episode_len_sec']*config.quadrotor_config['ctrl_freq'])

    # Start a timer.
    START = time.time()

    # Create controller.
    env_func = partial(make,
                    "quadrotor",
                    **config.quadrotor_config
                    )
    ctrl = make("rbcpid",
                env_func,
                **config.algo_config
                )

    if config.quadrotor_config.task == 'traj_tracking':
        reference_traj = ctrl.reference
        print(len(reference_traj))
        # Plot trajectory.
        for i in range(0, reference_traj.shape[0], 10):
            p.addUserDebugLine(lineFromXYZ=[reference_traj[i-10,0], 0, reference_traj[i-10,2]],
                                lineToXYZ=[reference_traj[i,0], 0, reference_traj[i,2]],
                                lineColorRGB=[1, 0, 0],
                                physicsClientId=ctrl.env.PYB_CLIENT)

    # Run the experiment.
    results = ctrl.run( iterations=ITERATIONS)
    ctrl.close()
    total_trajectory_loss = 0
    # Plot the experiment.
    for i in range(len(results['info'])):
        # Step the environment and print all returned information.
        obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], results['info'][i], results['action'][i]
        err = float(info[0]['mse'])
        total_trajectory_loss += err

    print("Total Loss PID " + str(total_trajectory_loss))
if __name__ == "__main__":
    main()