"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 ./pid_experiment.py --task quadrotor --algo pid --overrides ./config_pid_quadrotor.yaml

"""
import time
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
    ITERATIONS = int(config.quadrotor_config['episode_len_sec']*config.quadrotor_config['ctrl_freq'])
    
    # Start a timer.
    START = time.time()
    
    # Create controller.
    env_func = partial(make,
                    "quadrotor",
                    **config.quadrotor_config
                    )
    ctrl = make("rbcmpc",
                env_func,
                **config.algo_config
                )

    # Run the experiment.
    results = ctrl.run()
    ctrl.close()
    total_trajectory_loss = 0
    # Plot the experiment.
    for i in range(ITERATIONS):
        # Step the environment and print all returned information.
        obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], results['info'][
            i], results['action'][i]
        err = float(info[0]['mse'])
        total_trajectory_loss += err

    print("Total Loss MPC" + str(total_trajectory_loss))

if __name__ == "__main__":
    main()