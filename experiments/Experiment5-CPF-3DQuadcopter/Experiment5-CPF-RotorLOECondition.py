import random
import time
import cv2
import numpy as np
import os

from lero_control_gym.utils.configuration import ConfigFactory
from lero_control_gym.tuning.ClusteredParticleFiltering.CPF import ClusteredParticleFilteringOptimization as CPF

class Experiment():
    def __init__(self, experiment_config ):

        self.generalConfig = experiment_config
        random.seed(self.generalConfig.Experiment.random_seed)
        # self.generalConfig.Experiment.CPF.random_seed = random.randint(0,10000)
        return

    def createVideo(self):
        img_array = []
        for i in range(len(os.listdir("CPF-Images"))):  # (The only change I have made is here to the filepath.)
            img = cv2.imread('CPF-Images/'+str(i+1)+'.png')
            # print(i)
            height, width, layers = img.shape
            size = (width, height)
            img_array.append(img)

        out = cv2.VideoWriter('Exp5.avi', cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()

    def run(self):
        results = []
        for i in range(self.generalConfig.Experiment.numberIterations):
            self.generalConfig.Experiment.CPF.random_seed = random.randint(0, 10000)
            algo = CPF(**self.generalConfig.Experiment.CPF)
            result = algo.run()
            results.append(result)
            print(result)

        self.createVideo()
        # print(results)
        # p_average = 0
        # d_average = 0
        #
        # for result in results:
        #     p = result[0][0]
        #     d = result[0][1]
        #     p_average += p
        #     d_average += d
        #
        # p_average = p_average / len(results)
        # d_average = d_average / len(results)

        return results

def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()



    Exp = Experiment(config)
    print("===============================")
    START = time.time()
    print("Running CPF ")
    Exp = Experiment(config)
    result = Exp.run()
    END = time.time()
    print("Experiment took " + str(END - START) + " milli seconds")
    print("=============CPF Result==================")
    print(result)
    print("===============================")





if __name__ == "__main__":
    main()