import random
import numpy as np
from lero_control_gym.utils.configuration import ConfigFactory
from lero_control_gym.tuning.ParticleSwarmOptimization.PSO import ParticleSwarmOptimization as PSO

class Experiment():
    def __init__(self, experiment_config ):

        self.generalConfig = experiment_config
        random.seed(self.generalConfig.Experiment.random_seed)
        self.generalConfig.Experiment.PSO.random_seed = random.randint(0,10000)
        return

    def run(self):
        results = []
        for i in range(self.generalConfig.Experiment.numberIterations):
            self.generalConfig.Experiment.PSO.random_seed = random.randint(0, 10000)
            algo = PSO(**self.generalConfig.Experiment.PSO)
            result = algo.run()
            results.append(result)
            print(result)

        # print(results)
        p_average = 0
        d_average = 0

        for result in results:
            p = result[0][0]
            d = result[0][1]
            p_average += p
            d_average+=d

        p_average = p_average / len(results)
        d_average = d_average / len(results)


        return [p_average, d_average]

def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()
    print("===============================")
    print("Running PSO - Nominal Conditions")
    Exp = Experiment(config)
    result = Exp.run()
    print("PSO Result - Nominal Conditions")
    print(result)
    print("===============================")





if __name__ == "__main__":
    main()