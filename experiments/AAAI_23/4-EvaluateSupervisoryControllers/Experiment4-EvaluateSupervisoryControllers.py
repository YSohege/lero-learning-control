import random

import numpy as np

from lero_control_gym.utils.configuration import ConfigFactory

from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Uniform_RBC.Quadcopter3D_Uniform_RBC import Task as Uniform_RBC_Task
from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Optimal_Switching.Quadcopter3D_Optimal_Switching import Task as Optimal_Switching_Task
from lero_control_gym.tasks.AAAI_23.Quadcopter3D_Fixed_Distribution_RBC.Quadcopter3D_Fixed_Distribution_RBC import Task as Fixed_Distribution_RBC_Task
from lero_control_gym.tasks.AAAI_23.Quadcopter3D_RL_Distribution_RBC.Quadcopter3D_RL_Distribution_RBC import Task as RL_RBC_Task


class Experiment():
    def __init__(self, experiment_config ,
                        quadcopter3D ,
                 baseline_controller1,
                 baseline_controller2,
                 optimal_mmac_pid_0 ,
                 optimal_mmac_pid_1 ,
                 optimal_mmac_pid_2 ,
                 optimal_mmac_pid_3 ,
                 uniform_rbc_pid,
                 rl_rbc_pid,
                 fixed_dist_rbc_pid):
        self.generalConfig = experiment_config
        random.seed(self.generalConfig.random_seed)

        self.quadcopter_config = quadcopter3D
        self.quadcopter_config.Path.randomSeed = random.randint(0, 100000)

        self.baseline_controller1 = baseline_controller1
        self.baseline_controller2 = baseline_controller2
        self.optimal_mmac_pid_0 = optimal_mmac_pid_0
        self.optimal_mmac_pid_1 = optimal_mmac_pid_1
        self.optimal_mmac_pid_2 = optimal_mmac_pid_2
        self.optimal_mmac_pid_3 = optimal_mmac_pid_3
        # self.optimal_mmac_pid_4 = optimal_mmac_pid_4

        self.uniform_rbcpid = uniform_rbc_pid
        self.uniform_rbcpid.SEED = self.generalConfig.random_seed

        self.rl_rbc_pid = rl_rbc_pid

        self.fixed_dist_rbcpid = fixed_dist_rbc_pid
        self.fixed_dist_rbcpid.SEED = self.generalConfig.random_seed


        return



    def run(self):

        Baseline1 = []
        Baseline2 = []
        OptSwitch0 = []
        OptSwitch1 = []
        OptSwitch2 = []
        OptSwitch3 = []
        # OptSwitch4 = []
        UniRBC = []
        RLRBC = []
        FixedRBC = []

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
            #
            optimal_switching_task0 = Optimal_Switching_Task(self.quadcopter_config, self.optimal_mmac_pid_0)
            optimal_switching_result0 = optimal_switching_task0.executeTask()
            OptSwitch0.append(optimal_switching_result0)

            optimal_switching_task1 = Optimal_Switching_Task(self.quadcopter_config, self.optimal_mmac_pid_1)
            optimal_switching_result1 = optimal_switching_task1.executeTask()
            OptSwitch1.append(optimal_switching_result1)

            optimal_switching_task2 = Optimal_Switching_Task(self.quadcopter_config, self.optimal_mmac_pid_2)
            optimal_switching_result2 = optimal_switching_task2.executeTask()
            OptSwitch2.append(optimal_switching_result2)

            optimal_switching_task3 = Optimal_Switching_Task(self.quadcopter_config, self.optimal_mmac_pid_3)
            optimal_switching_result3 = optimal_switching_task3.executeTask()
            OptSwitch3.append(optimal_switching_result3)

            uniform_rbc_task = Uniform_RBC_Task(self.quadcopter_config, self.uniform_rbcpid)
            uniform_rbc_result = uniform_rbc_task.executeTask()
            UniRBC.append(uniform_rbc_result)

            rl_rbc_task = RL_RBC_Task(self.quadcopter_config, self.rl_rbc_pid)
            rl_rbc_result = rl_rbc_task.executeTask()
            RLRBC.append(rl_rbc_result)

            fixed_dist_rbc_task = Fixed_Distribution_RBC_Task(self.quadcopter_config, self.fixed_dist_rbcpid)
            fixed_dist_rbc_result = fixed_dist_rbc_task.executeTask()
            FixedRBC.append(fixed_dist_rbc_result)

            print("Baseline Controller 1 ")
            print(np.average(Baseline1))

            print("Baseline Controller 2 ")
            print(np.average(Baseline2))

            print("Opt Switching Average - "+str(self.optimal_mmac_pid_0.SWITCH_DELAY)+" delay ")
            print(np.average(OptSwitch0))

            print("Opt Switching Average -  "+str(self.optimal_mmac_pid_1.SWITCH_DELAY)+" delay ")
            print(np.average(OptSwitch1))

            print("Opt Switching Average -  "+str(self.optimal_mmac_pid_2.SWITCH_DELAY)+" delay ")
            print(np.average(OptSwitch2))

            print("Opt Switching Average - "+str(self.optimal_mmac_pid_3.SWITCH_DELAY)+" delay ")
            print(np.average(OptSwitch3))

            print("Uni RBC Average")
            print(np.average(UniRBC))

            print("RL RBC Average")
            print(np.average(RLRBC))

            print("Fixed RBC Average")
            print(np.average(FixedRBC))
            print("-------------"+ str(i+1)+"/"+str(self.generalConfig.number_iterations)+"----------")


            results = [Baseline1,
                       Baseline2,
                       OptSwitch0,
                       OptSwitch1,
                       OptSwitch2,
                       OptSwitch3,
                       UniRBC,
                       RLRBC,
                       FixedRBC]

        return results

def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()

    Experiment1 = Experiment(**config.Experiment)
    experimental_results = Experiment1.run()
    # print(experimental_results)

    return



if __name__ == "__main__":
    main()