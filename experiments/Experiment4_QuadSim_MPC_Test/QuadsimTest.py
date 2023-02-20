"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import json
import time
import numpy as np

from functools import partial
import random
from munch import munchify,unmunchify
from lero_control_gym.utils.configuration import ConfigFactory
from lero_control_gym.utils.registration import make
import os




def main():
    """The main function creating, running, and closing an environment.

    """

    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()


    #UPDATE ENV CONDITIONS HERE

    env_func = partial(make,
                       'quadsimD',
                       **config.quadsim
                       )


    ctrl = make("quadsimnonlinearmpc",
                env_func,
                **config.nonlinearMPC
                )


    results = ctrl.run()
    exit()
    ctrl.close()


if __name__ == "__main__":
    main()