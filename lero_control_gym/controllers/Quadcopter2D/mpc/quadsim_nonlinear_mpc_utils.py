import numpy as np
import os


# Note that before running, these LD Library Path needs to be configured
def check_mpc_hsl_solver_in_path():
    # os.path.dirname(os.path.abspath(__file__)) +
    hsl_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) +
                               "/../src/controllers/hsl/lib")

    if not any(
        hsl_path in x
            for x in os.environ['LD_LIBRARY_PATH'].split(':')):

        print("MPC HSL Solver in LD_LIBRARY_PATH does not found.")
        print(
            "Please run this command in terminal at " +
            "working directory root folder:"
        )
        print(hsl_path)
        print(
            r'export LD_LIBRARY_PATH=' +
            r'"$LD_LIBRARY_PATH:' + hsl_path + r'"')
        exit(1)
