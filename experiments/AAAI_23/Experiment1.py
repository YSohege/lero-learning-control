import random

import numpy as np

from lero_control_gym.utils.configuration import ConfigFactory

from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Uniform_RBC.Quadcopter3D_Uniform_RBC import Task as Uniform_RBC_Task
from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Optimal_Switching.Quadcopter3D_Optimal_Switching import Task as Optimal_Switching_Task
from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Fixed_Distribution_RBC.Quadcopter3D_Fixed_Distribution_RBC import Task as Fixed_Distribution_RBC_Task


class Experiment():
    def __init__(self, experiment_config , quadcopter3D , optimal_mmac_pid , uniform_rbc_pid, fixed_dist_rbc_pid):
        self.generalConfig = experiment_config
        random.seed(self.generalConfig.random_seed)

        self.quadcopter_config = quadcopter3D

        self.optimal_mmac_pid = optimal_mmac_pid

        self.uniform_rbcpid = uniform_rbc_pid
        self.uniform_rbcpid.SEED = self.generalConfig.random_seed

        self.fixed_dist_rbcpid = fixed_dist_rbc_pid
        self.fixed_dist_rbcpid.SEED = self.generalConfig.random_seed


        return

    def run(self):

        OptSwitch = []
        UniRBC = []
        FixedRBC = []

        for i in range(self.generalConfig.number_iterations):


            optimal_switching_task = Optimal_Switching_Task(self.quadcopter_config, self.optimal_mmac_pid)
            optimal_switching_result = optimal_switching_task.executeTask()
            # print("optimal_switching_result")
            # print(optimal_switching_result)
            OptSwitch.append(optimal_switching_result)

            uniform_rbc_task = Uniform_RBC_Task(self.quadcopter_config, self.uniform_rbcpid)
            uniform_rbc_result = uniform_rbc_task.executeTask()
            # print("uniform_rbc_result")
            # print(uniform_rbc_result)
            UniRBC.append(uniform_rbc_result)

            fixed_dist_rbc_task = Fixed_Distribution_RBC_Task(self.quadcopter_config, self.fixed_dist_rbcpid)
            fixed_dist_rbc_result = fixed_dist_rbc_task.executeTask()
            # print("fixed_dist_rbc_result")
            # print(fixed_dist_rbc_result)
            FixedRBC.append(fixed_dist_rbc_result)

        print("Opt Average")
        print(np.average(OptSwitch))

        print("Uni RBC Average")
        print(np.average(UniRBC))

        print("Fixed RBC Average")
        print(np.average(FixedRBC))


        results = [OptSwitch, UniRBC, FixedRBC]
        return

def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()

    Experiment1 = Experiment(**config.Experiment)
    experimental_results = Experiment1.run()
    print(experimental_results)

    return



if __name__ == "__main__":
    main()