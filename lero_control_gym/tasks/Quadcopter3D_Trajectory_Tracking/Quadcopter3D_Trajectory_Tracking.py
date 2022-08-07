"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import time
from functools import partial
from lero_control_gym.utils.registration import make


class Task():

    def __init__(self, quadcopter3D , mmac_pid ):
        self.quadcopter = quadcopter3D
        self.controller = mmac_pid

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


        ctrl = make("quad3d_mmacpid",
                    env_func,
                    **self.controller
                    )

        # Run the experiment.
        results = ctrl.run(iterations=STEPS)

        total_trajectory_loss = 0
        # Plot the experiment.
        for i in range(len(results['info'])):
            # Step the environment and print all returned information.
            obs, reward, done, info, action = results['obs'][i], results['reward'][i], results['done'][i], results['info'][
                i], results['action'][i]
            err = float(info[0]['mse'])
            total_trajectory_loss += err

        total_trajectory_loss = -total_trajectory_loss

        END = time.time()
        ctrl.close()
        return total_trajectory_loss



# if __name__ == "__main__":
#     main()