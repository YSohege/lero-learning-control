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



def main(case = 1):
    """The main function creating, running, and closing an environment.

    """

    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()
    # Create controller.
    env_func = make('test_speed', config.numIter)

    if case ==1 :
        env_func.runNumpy()

    if case ==2 :
        env_func.runCPU()

    if case == 3:
        env_func.runGPU()

    # Run the experiment.


    return







if __name__ == "__main__":

    result = timeit.timeit(stmt='main(1)', globals=globals(), number=1)
    print(f"NUMPY ON CPU: Execution time is {result} seconds")

    result = timeit.timeit(stmt='main(2)', globals=globals(), number=1)
    print(f"TENSOR ON CPU: Execution time is {result} seconds")

    result = timeit.timeit(stmt='main(3)', globals=globals(), number=1)
    print(f"TENSOR ON GPU: Execution time is {result} seconds")