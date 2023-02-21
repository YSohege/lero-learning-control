import random
import matplotlib.pyplot as plt
import numpy as np

from lero_control_gym.utils.configuration import ConfigFactory

from lero_control_gym.tasks.Quadcopter3D_Uniform_RBC.Quadcopter3D_Uniform_RBC import Task as Uniform_RBC_Task
from lero_control_gym.tasks.Quadcopter3D_Optimal_Switching.Quadcopter3D_Optimal_Switching import Task as Optimal_Switching_Task
from lero_control_gym.tasks.Quadcopter3D_Fixed_Distribution_RBC.Quadcopter3D_Fixed_Distribution_RBC import Task as Fixed_Distribution_RBC_Task
from lero_control_gym.tasks.Quadcopter3D_RL_Distribution_RBC.Quadcopter3D_RL_Distribution_RBC import Task as RL_RBC_Task


class Experiment():
    def __init__(self, experiment_config,
                 quadcopter3D,
                 baseline_controller1,
                 baseline_controller2,
                 optimal_mmac_pid_0,
                 optimal_mmac_pid_1,
                 optimal_mmac_pid_2,
                 optimal_mmac_pid_3,
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
        NN_average_output = [[], []]
        for i in range(self.generalConfig.number_iterations):
            self.quadcopter_config.Path.randomSeed = random.randint(0, 100000)
            faultStart = random.randint(0, 1000)
            self.quadcopter_config.Env.RotorFault.starttime = faultStart
            self.quadcopter_config.Env.Wind.starttime = faultStart
            self.quadcopter_config.Env.PositionNoise.starttime = faultStart
            self.quadcopter_config.Env.AttitudeNoise.starttime = faultStart

            # baseline_task1 = Optimal_Switching_Task(self.quadcopter_config, self.baseline_controller1)
            # baseline_result1 = baseline_task1.executeTask()
            # Baseline1.append(baseline_result1)
            #
            # baseline_task2 = Optimal_Switching_Task(self.quadcopter_config, self.baseline_controller2)
            # baseline_result2 = baseline_task2.executeTask()
            # Baseline2.append(baseline_result2)
            # #
            optimal_switching_task0 = Optimal_Switching_Task(
                self.quadcopter_config, self.optimal_mmac_pid_0)
            optimal_switching_result0 = optimal_switching_task0.executeTask()
            OptSwitch0.append(optimal_switching_result0)

            optimal_switching_task1 = Optimal_Switching_Task(
                self.quadcopter_config, self.optimal_mmac_pid_1)
            optimal_switching_result1 = optimal_switching_task1.executeTask()
            OptSwitch1.append(optimal_switching_result1)

            optimal_switching_task2 = Optimal_Switching_Task(
                self.quadcopter_config, self.optimal_mmac_pid_2)
            optimal_switching_result2 = optimal_switching_task2.executeTask()
            OptSwitch2.append(optimal_switching_result2)

            optimal_switching_task3 = Optimal_Switching_Task(
                self.quadcopter_config, self.optimal_mmac_pid_3)
            optimal_switching_result3 = optimal_switching_task3.executeTask()
            OptSwitch3.append(optimal_switching_result3)

            uniform_rbc_task = Uniform_RBC_Task(
                self.quadcopter_config, self.uniform_rbcpid)
            uniform_rbc_result = uniform_rbc_task.executeTask()
            UniRBC.append(uniform_rbc_result)

            rl_rbc_task = RL_RBC_Task(self.quadcopter_config, self.rl_rbc_pid)
            rl_rbc_result, average_action = rl_rbc_task.executeTask()
            RLRBC.append(rl_rbc_result)
            NN_average_output[0].append(average_action[0])
            NN_average_output[1].append(average_action[1])

            fixed_dist_rbc_task = Fixed_Distribution_RBC_Task(
                self.quadcopter_config, self.fixed_dist_rbcpid)
            fixed_dist_rbc_result = fixed_dist_rbc_task.executeTask()
            FixedRBC.append(fixed_dist_rbc_result)

            # print("Baseline Controller 1 mean, std ")
            # print(np.mean(Baseline1))
            # print(np.std(Baseline1))
            #
            # print("Baseline Controller 2 mean, std ")
            # print(np.mean(Baseline2))
            # print(np.std(Baseline2))

            # print("Opt Switching mean, std - "+str(self.optimal_mmac_pid_0.SWITCH_DELAY)+" delay ")
            # print(np.mean(OptSwitch0))
            # print(np.std(OptSwitch0))
            #
            # print("Opt Switching mean, std -  "+str(self.optimal_mmac_pid_1.SWITCH_DELAY)+" delay ")
            # print(np.mean(OptSwitch1))
            # print(np.std(OptSwitch1))
            #
            # print("Opt Switching mean, std -  "+str(self.optimal_mmac_pid_2.SWITCH_DELAY)+" delay ")
            # print(np.mean(OptSwitch2))
            # print(np.std(OptSwitch2))
            #
            # print("Opt Switching mean, std - "+str(self.optimal_mmac_pid_3.SWITCH_DELAY)+" delay ")
            # print(np.mean(OptSwitch3))
            # print(np.std(OptSwitch3))
            #
            # print("Uni RBC mean, std")
            # print(np.mean(UniRBC))
            # print(np.std(UniRBC))
            #
            # print("RL RBC mean, std")
            # print(np.mean(RLRBC))
            # print(np.std(RLRBC))
            # print(np.average(NN_average_output[0]), np.average(NN_average_output[1]))
            #
            # print("Fixed RBC mean, std")
            # print(np.mean(FixedRBC))
            # print(np.std(FixedRBC))
            # print("-------------"+ str(i+1)+"/"+str(self.generalConfig.number_iterations)+"----------")

            results = [
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
    fig = plt.figure(figsize=(12, 7))
    ax = fig.add_subplot(111)

    labels = (
        '',
        'Trad-0',
        'Trad-1',
        'Trad-2',
        'Trad-3',
        'RBC-Uni',
        'RBC-Man',
        'RBC-RL')
    y_pos = np.arange(len(labels))
    performance = [
        np.mean(experimental_results[0]),
        np.mean(experimental_results[1]),
        np.mean(experimental_results[2]),
        np.mean(experimental_results[3]),
        np.mean(experimental_results[4]),
        np.mean(experimental_results[5]),
        np.mean(experimental_results[6])
    ]

    plt.bar(
        y_pos,
        performance,
        align='center',
        alpha=0.8,
        color=[
            'grey',
            'deepskyblue',
            'darkorange',
            'seagreen',
            'purple',
            'lightcoral',
            'gold'])
    # plt.boxplot(experimental_results)
    plt.rcParams.update({'font.size': 16})
    print(experimental_results)
    plt.xticks(y_pos, labels, fontsize=16)

    plt.ylabel('Operational Volume Loss - (1000 eps)', fontsize=16)

    # plt.gca().set_ylim([0, -400])
    plt.title("Supervisory Controller Comparison - Rotor LOE 30%")
    plt.savefig('RotorLOE.png')
    return


if __name__ == "__main__":
    main()
