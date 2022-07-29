"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import time
from functools import partial

from lero_control_gym.utils.configuration import ConfigFactory
from lero_control_gym.utils.registration import make
from lero_control_gym.tuning.ParticleSwarmOptimization.PSO import ParticleSwarmOptimization


def main():
    """The main function creating, running, and closing an environment.

    """

    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()
    print(config)
    PSO = ParticleSwarmOptimization(**config.PSO)


    result = PSO.run()

    print("Optimal Tuning" + str(result))




if __name__ == "__main__":
    main()