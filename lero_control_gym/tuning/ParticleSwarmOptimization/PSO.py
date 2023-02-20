import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import datetime
import os
import random
from functools import partial
from lero_control_gym.utils.registration import make
from lero_control_gym.tasks.Quadcopter3D_Trajectory_Tracking.Quadcopter3D_Trajectory_Tracking import Task
from lero_control_gym.utils.configuration import ConfigFactory


class ParticleSwarmOptimization():
    DefaultEnv = []
    DefaultController = []
    DefaultRanges = []

    def __init__(self,
                 random_seed=123,
                 NumberParticles=50,
                 c1=3,
                 c2=2,
                 Wmin=0.05,
                 Wmax=1,
                 w=1,
                 maxVelocity=1000,
                 ParameterRanges=DefaultRanges,
                 Env=DefaultEnv,
                 Controller=DefaultController,
                 render=True):

        random.seed(random_seed)
        # print(NumberParticles)
        self.NumberParticles = NumberParticles
        self.c1 = c1
        self.c2 = c2
        self.Wmin = Wmin
        self.Wmax = Wmax
        self.w = w
        self.maxVelocity = maxVelocity
        self.ParameterRanges = ParameterRanges

        self.controllerConfig = Controller
        self.env = Env
        self.databaseConfig = {
            "NewFile": True,
            "folder": "./results/"
        }
        if bool(self.databaseConfig['NewFile']):
            dt = str(datetime.datetime.now())[:16].replace(":", "-")
            FileName = self.databaseConfig['folder'] + \
                "PSO_Results-" + dt + ".json"
            # print(FileName)
            if not os.path.exists(FileName):
                with open(FileName, 'w') as db:
                    json.dump([], db)

            self.databaseConfig['path'] = FileName
        self.database = []

        self.Params = []
        # internal swarm representation
        self.particles = [0] * self.NumberParticles
        self.velocities = [0] * self.NumberParticles
        self.localBests = [0] * self.NumberParticles
        self.globalBest = None
        self.render = render
        self.particlePaths = [[]] * self.NumberParticles
        self.numberDims = 0
        if self.render:

            self.numberDims += 1 if (self.ParameterRanges.P.max -
                                     self.ParameterRanges.P.min) > 0 else 0
            self.numberDims += 1 if (self.ParameterRanges.I.max -
                                     self.ParameterRanges.I.min) > 0 else 0
            self.numberDims += 1 if (self.ParameterRanges.D.max -
                                     self.ParameterRanges.D.min) > 0 else 0
            if self.numberDims > 2:
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection='3d')
                self.ax.set_xlim(
                    self.ParameterRanges.P.min,
                    self.ParameterRanges.P.max)
                self.ax.set_ylim(
                    self.ParameterRanges.I.min,
                    self.ParameterRanges.I.max)
                self.ax.set_zlim(
                    self.ParameterRanges.D.min,
                    self.ParameterRanges.D.max)
                self.ax.set_xlabel("P")
                self.ax.set_ylabel("I")
                self.ax.set_zlabel("D")
            else:
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111)
                self.ax.set_xlim(
                    self.ParameterRanges.P.min,
                    self.ParameterRanges.P.max)
                self.ax.set_ylim(
                    self.ParameterRanges.D.min,
                    self.ParameterRanges.D.max)
                self.ax.set_xlabel("P")
                self.ax.set_ylabel("D")
        self.initilizeParticles()

        self.finished = False
        return

    def initilizeParticles(self):

        for i in range(self.NumberParticles):

            numParameters = 0
            params = []

            for param in self.ParameterRanges:
                max = self.ParameterRanges[param].max
                min = self.ParameterRanges[param].min
                if max - min > 0:
                    numParameters += 1
                    params.append(param)

            self.Params = params
            particleParams = [0] * numParameters
            particleVelocity = [0] * numParameters

            paramNumber = 0
            for param in params:
                Max = self.ParameterRanges[param].max
                Min = self.ParameterRanges[param].min
                vel = self.maxVelocity
                if max - min > 0:
                    particleParams[paramNumber] = np.random.uniform(Min, Max)
                    particleVelocity[paramNumber] = np.random.uniform(
                        -vel, vel)
                    paramNumber += 1

            self.particles[i] = particleParams
            self.particlePaths[i] = [particleParams]
            self.velocities[i] = particleVelocity
            # performance = self.evaluateParticle(particleParams)
            # performance = self.evaluateParticle(particleParams)
            self.localBests[i] = (
                particleParams, -(self.env['Path']['maxStepsPerRun'] + 1))
            # if self.globalBest == None or performance > self.globalBest[1]:
        self.globalBest = (
            self.particles[i], -(self.env['Path']['maxStepsPerRun'] + 1))

        if self.render:
            self.renderCount = 0
            self.render3D()

        return

    def evaluateParticle(self, controller):

        controlConfig = self.controllerConfig
        # override the pid controllers
        # print(controller)
        if len(controller) > 2:

            controlConfig.ATTITUDE_CONTROLLER_SET = [
                [
                    [controller[0], controller[0], 1500],
                    [controller[1], controller[1], 1.2],
                    [controller[2], controller[2], 0]
                ]
            ]
        else:
            controlConfig.ATTITUDE_CONTROLLER_SET = [
                [
                    [controller[0], controller[0], 1500],
                    [0, 0, 1.2],
                    [controller[1], controller[1], 0]
                ]
            ]

        quadConfig = self.env
        quadConfig.Path.randomSeed = random.randint(0, 100000)
        task = Task(quadConfig, controlConfig)

        results = task.executeTask()

        # print("Total Loss PID " + str(results))

        # print(str(controller) + " " + str(particlePerformance))
        self.saveResult(controlConfig, quadConfig.Env.toDict(), results)
        return results

    def saveResult(self, controllerParameters, quadconf, particlePerformance):

        datapoint = {'ControllerParameters': controllerParameters,
                     'Environment': quadconf,
                     'performance': particlePerformance}
        self.database.append(datapoint)
        # print(self.database)
        data = []
        with open(self.databaseConfig['path'], 'r') as db:
            try:
                data = json.load(db)
            except Exception:
                data = []
        data.append(datapoint)
        # print(len(self.database))
        # print(len(data))

        with open(self.databaseConfig['path'], 'w') as db:
            json.dump(data, db)
        return

    def run(self):
        # print("PSO-Images-Nominal-2 running")
        self.currentWeight = self.w
        while not self.finished:
            for i in range(len(self.particles)):

                pos = self.particles[i]

                performance = self.evaluateParticle(pos)

                # print(str(pos) + " fitness: " + str(fitness))
                if performance > self.localBests[i][1]:
                    self.localBests[i] = (pos, performance)

                if performance > self.globalBest[1]:
                    self.globalBest = (pos, performance)

            # update interia
            for i in range(len(self.particles)):
                r1 = 1
                r2 = np.random.uniform(0.0, 1.0) * self.c1
                r3 = np.random.uniform(0.0, 1.0) * self.c2

                # [r1 * self.velocities[i][0], r1 * self.velocities[i][1]]
                InteriaComp = [min(max(r1 * v, -self.maxVelocity),
                                   self.maxVelocity) for v in self.velocities[i]]

                CognitiveComp = [0] * len(self.particles[i])
                for j in range(len(self.localBests[i][0])):
                    localBest = float(self.localBests[i][0][j])
                    particleParam = float(self.particles[i][j])
                    cognitiveVelocity = localBest - particleParam
                    cog = min(max(r2 * (cognitiveVelocity),
                                  -self.maxVelocity), self.maxVelocity)
                    CognitiveComp[j] = cog

                SocialComp = [0] * len(self.particles[i])

                for k in range(len(self.globalBest[0])):
                    globalBest = float(self.globalBest[0][k])
                    particleParam = float(self.particles[i][k])
                    socialVelocity = globalBest - particleParam
                    soc = min(max(r3 * (socialVelocity),
                                  -self.maxVelocity), self.maxVelocity)
                    SocialComp[k] = soc

                newVel = [0] * len(self.particles[i])
                # print(self.particles[i])
                for param in range(len(newVel)):
                    newVel[param] = (
                        InteriaComp[param] + CognitiveComp[param] + SocialComp[param]) * self.currentWeight

                self.velocities[i] = newVel

                newPos = [0] * len(self.Params)

                p = 0
                for param in self.Params:
                    minimum = self.ParameterRanges[param].min
                    maximum = self.ParameterRanges[param].max
                    if maximum - minimum > 0:
                        parameterPosition = self.particles[i][p]
                        parameterVelocity = self.velocities[i][p]
                        newPosition = parameterPosition + parameterVelocity
                        newPos[p] = min(max(newPosition, minimum), maximum)
                        p += 1

                self.particles[i] = newPos
                self.particlePaths[i].append(self.particles[i])
            # print("Global Best = " + str(self.globalBest) + " at " + str(currentWeight))

            if self.currentWeight > self.Wmin:
                self.currentWeight -= 0.01
                # print(currentWeight)
            else:
                self.finished = True

            if self.render:
                self.render3D()
        return self.globalBest

    def render3D(self):
        self.renderCount += 1
        if self.render:
            self.ax.cla()
            Env = "Nominal"
            faults = self.env.Env.toDict()
            if faults['RotorFault']['enabled']:
                Env = "Rotor Fault - " + str(faults['RotorFault']['magnitude'])

            self.ax.set_title("Particle Swarm Optimization -" +
                              Env + "- iteration:" + str(self.renderCount))
            xdata = []
            ydata = []
            zdata = []
            sizes = []
            colors = []
            for i in range(self.NumberParticles):
                xdata.append(self.particles[i][0])
                ydata.append(self.particles[i][1])
                # zdata.append(self.particles[i][2])
                sizes.append(50)

            if len(self.particlePaths[0]) > 1:
                for particleHistory in self.particlePaths:
                    xPath = []
                    yPath = []
                    # zPath = []
                    ind = len(
                        self.particlePaths[0]) if len(
                        self.particlePaths[0]) <= 5 else 5
                    particleHistory = particleHistory[-5:] if len(
                        particleHistory) > 5 else particleHistory
                    for point in particleHistory:
                        xPath.append(point[0])
                        yPath.append(point[1])
                        # zPath.append(point[2])

                    # self.ax.plot(xPath, yPath, zPath)
                    self.ax.plot(xPath, yPath)

            # self.ax.scatter(xdata, ydata, zdata, s=sizes, alpha=0.8, c='k', edgecolor="k", linewidth=0.4)
            # self.ax.set_xlim(self.ParameterRanges.P.min, self.ParameterRanges.P.max)
            # self.ax.set_ylim(self.ParameterRanges.I.min, self.ParameterRanges.I.max)
            # self.ax.set_zlim(self.ParameterRanges.D.min, self.ParameterRanges.D.max)
            # self.ax.set_xlabel("P")
            # self.ax.set_ylabel("I")
            # self.ax.set_zlabel("D")

            self.ax.scatter(
                xdata,
                ydata,
                s=sizes,
                alpha=0.8,
                c='k',
                edgecolor="k",
                linewidth=0.4)
            self.ax.set_xlim(
                self.ParameterRanges.P.min,
                self.ParameterRanges.P.max)
            # self.ax.set_lim(self.ParameterRanges.I.min, self.ParameterRanges.I.max)
            self.ax.set_ylim(
                self.ParameterRanges.D.min,
                self.ParameterRanges.D.max)
            self.ax.set_xlabel("P")
            # self.ax.set_ylabel("I")
            self.ax.set_ylabel("D")
            plt.draw()

            plt.pause(0.0001)
            plt.savefig("PSO-Images/" + str(self.renderCount) + ".png")

        return


def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()
    print(config)

    random.seed(config.random_seed)
    results = []
    for i in range(config.numberIterations):
        algo = ParticleSwarmOptimization(**config.PSO)
        result = algo.run()
        results.append(result)
        print(result)

    print(results)
    p_average = 0
    i_average = 0
    d_average = 0

    for result in results:
        p = result[0][0]
        i = result[0][1]
        # d = result[0][2]
        p_average += p
        i_average += i
        # d_average+=d

    p_average = p_average / len(results)
    i_average = i_average / len(results)
    # d_average = d_average/len(results)

    print(p_average)
    print(i_average)

    return


if __name__ == "__main__":
    main()
