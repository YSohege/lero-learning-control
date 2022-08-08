import random

import numpy as np

from lero_control_gym.utils.configuration import ConfigFactory
from scipy import stats
from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Uniform_RBC.Quadcopter3D_Uniform_RBC import Task as Uniform_RBC_Task
from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Optimal_Switching.Quadcopter3D_Optimal_Switching import Task as Optimal_Switching_Task


class Experiment():
    def __init__(self, experiment_config ,
                        quadcopter3D ,
                 baseline_controller1,
                 baseline_controller2,
                 uniform_rbc_pid
                 ):
        self.generalConfig = experiment_config
        random.seed(self.generalConfig.random_seed)

        self.quadcopter_config = quadcopter3D
        self.quadcopter_config.Path.randomSeed = random.randint(0, 100000)

        self.baseline_controller1 = baseline_controller1
        self.baseline_controller2 = baseline_controller2


        self.uniform_rbcpid = uniform_rbc_pid
        self.uniform_rbcpid.SEED = self.generalConfig.random_seed

        return

    def run(self):

        Baseline1 = []
        Baseline2 = []
        # UniRBC = []


        for i in range(self.generalConfig.number_iterations):
            self.quadcopter_config.Path.randomSeed = random.randint(0, 100000)
            faultStart = random.randint(0, 1000)
            self.quadcopter_config.Env.RotorFault.starttime = faultStart
            self.quadcopter_config.Env.Wind.starttime = faultStart
            self.quadcopter_config.Env.PositionNoise.starttime = faultStart
            self.quadcopter_config.Env.AttitudeNoise.starttime = faultStart

            baseline_task1 = Optimal_Switching_Task(self.quadcopter_config, self.baseline_controller1)
            baseline_result1 = baseline_task1.executeTask()
            Baseline1.append(baseline_result1)

            baseline_task2 = Optimal_Switching_Task(self.quadcopter_config, self.baseline_controller2)
            baseline_result2 = baseline_task2.executeTask()
            Baseline2.append(baseline_result2)

            # uniform_rbc_task = Uniform_RBC_Task(self.quadcopter_config, self.uniform_rbcpid)
            # uniform_rbc_result = uniform_rbc_task.executeTask()
            # UniRBC.append(uniform_rbc_result)


            # print("Baseline Controller 1 ")
            # print(np.average(Baseline1))
            #
            # print("Baseline Controller 2 ")
            # print(np.average(Baseline2))
            #
            # print("Uni RBC Average")
            # print(np.average(UniRBC))


            results = [Baseline1,
                       Baseline2,
                       # UniRBC
                   ]

        wilcoxon_results = stats.wilcoxon(results[0], results[1])
        # print("Iteration - Nominal- "+ str(i)+ "/"+ str(self.generalConfig.number_iterations))
        # print("Controller 1 , Controller 2, Uni RBC")
        # print(results)
        # print("----------------")
        return wilcoxon_results, results

def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()
    print("----------------")
    print("Comparison on Nominal Conditions - Running")
    Experiment1 = Experiment(**config.Experiment)
    wilcoxon_results, experimental_results = Experiment1.run()
    print(wilcoxon_results)
    print("Baseline Controller 1 -  MEAN, STD")
    print( np.mean(experimental_results[0]), np.std(experimental_results[0]))
    print("Baseline Controller 2 -  MEAN, STD")
    print( np.mean(experimental_results[1]), np.std(experimental_results[1]))

    # print(experimental_results)

    return



if __name__ == "__main__":
    main()