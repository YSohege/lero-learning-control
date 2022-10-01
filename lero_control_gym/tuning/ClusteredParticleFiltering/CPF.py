import json
import time
from matplotlib import cm
import numpy as np
import random
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
import datetime
import os
import matplotlib.gridspec as gridspec
from lero_control_gym.tasks.Quadcopter3D_Trajectory_Tracking.Quadcopter3D_Trajectory_Tracking import Task
from lero_control_gym.utils.configuration import ConfigFactory

class ClusteredParticleFilteringOptimization():
    DefaultTask = []
    DefaultController = []
    DefaultRanges = []

    def __init__(self,
                 NumberParticlesPerDimension= 20,
                 numberOfParticlesToSample= 50,
                 averagePerformanceLength= 10,
                 MaxIter= 100,
                 StartingPerformance= -3000,
                 performanceThreshold= -1000,
                 min_samples = 4,
                 epsilon= 2000,
                 ParameterRanges=DefaultRanges,
                 Task=DefaultTask,
                 Controller=DefaultController,
                 random_seed = None,
                 render=False):
        if random_seed != None:
            random.seed(random_seed)

        self.NumberParticlesPerDimension = NumberParticlesPerDimension
        self.numberOfParticlesToSample = numberOfParticlesToSample
        self.averagePerformanceLength = averagePerformanceLength
        self.MaxIter = MaxIter
        self.StartingPerformance = StartingPerformance
        self.performanceThreshold = performanceThreshold
        self.ParameterRanges = ParameterRanges
        self.min_sample = min_samples
        self.epsilon = epsilon
        self.controllerConfig = Controller
        self.Task = Task
        self.databaseConfig = {
            "NewFile": True,
            "folder": "./results/"
        }
        if bool(self.databaseConfig['NewFile']):
            dt = str(datetime.datetime.now())[:16].replace(":", "-")
            FileName = self.databaseConfig['folder'] + "CPF_Results-" + dt + ".json"
            # print(FileName)
            if not os.path.exists(FileName):
                with open(FileName, 'w') as db:
                    json.dump([], db)

            self.databaseConfig['path'] = FileName
        self.database = []


        self.IntermediateHulls = []
        self.FinalHull = []
        self.currentHull = []

        # equal probability for all particles at the start



        # self.initilizeParticles()

        self.render = render
        if self.render:
            self.fig = plt.figure(figsize=(8,6))
            self.ax = plt.subplot(111, projection='3d')
            # self.ax = plt.subplot(111)
            # self.ax1 = plt.subplot(222)
            self.ax.set_xlim(self.ParameterRanges.P.min, self.ParameterRanges.P.max)
            # self.ax.set_ylim(self.CPFConfig['ParameterRanges']["I"]['min'], self.CPFConfig['ParameterRanges']["I"]['max'])
            self.ax.set_ylim(self.ParameterRanges.D.min, self.ParameterRanges.D.max)
            # self.ax1.set_xlim(self.CPFConfig['ParameterRanges']["P"]['min'], self.CPFConfig['ParameterRanges']["P"]['max'])
            # # self.ax1.set_ylim(self.CPFConfig['ParameterRanges']["I"]['min'], self.CPFConfig['ParameterRanges']["I"]['max'])
            # self.ax1.set_ylim(self.CPFConfig['ParameterRanges']["D"]['min'], self.CPFConfig['ParameterRanges']["D"]['max'])
            self.cbar = None
        self.finished = False
        return

    def initilizeParticles(self):

        self.particles = []
        self.performances = []
        self.avgPerformances = []
        self.weights = []
        self.totalPerformance = 0
        self.numberRuns = 0
        self.clusterPoints = []

        # startPerf =  self.StartingPerformance

        numParticles = self.NumberParticlesPerDimension
        P_min = int(self.ParameterRanges.P.min)
        P_max =int(self.ParameterRanges.P.max)
        P_step = round((P_max-P_min)/numParticles)
        # I_min =self.CPFConfig['ParameterRanges']["I"]['min']
        # I_max =self.CPFConfig['ParameterRanges']["I"]['max']
        # I_step =  round((I_max - I_min) / numParticles)
        D_min =int(self.ParameterRanges.D.min)
        D_max =int(self.ParameterRanges.D.max)
        D_step = round((D_max - D_min) / numParticles)
        for p in range(P_min, P_max, P_step):
            # for i  in range(I_min, I_max, I_step):
                for d  in range(D_min, D_max, D_step):
                    # self.particles.append((p, i, d))
                    # startPerf = random.uniform(self.StartingPerformance, -500)
                    startPerf = self.StartingPerformance
                    self.particles.append((p, d))
                    self.performances.append([startPerf])
                    self.avgPerformances.append(startPerf)

        for _ in self.particles:
            self.weights.append(1 / len(self.particles))
        self.finished = False
        return


    def setTaskEnv(self,env):
        if env != None:
            self.activeEnv = env
            self.TaskEnv = self.Task.Env[self.activeEnv]
        else:
            self.activeEnv = None
            self.TaskEnv = None
        return

    def setRandomEnvironmentMagnitude(self):
        # self.TaskEnv = self.Environment[env]
        if self.TaskEnv != None:
            min = self.TaskEnv.min_magnitude
            max = self.TaskEnv.max_magnitude
            self.TaskEnv.magnitude = random.uniform(min,max)

    # def setFixedEnvironment(self, percent):
    #     min = self.TaskEnv['min_magnitude']
    #     max = self.TaskEnv['max_magnitude']
    #     val = min+ ((max - min)*percent)
    #     self.TaskEnv['magnitude'] = val
    #     return

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

        quadConfig = self.Task
        quadConfig.Env[self.activeEnv] = self.TaskEnv
        quadConfig.Path.randomSeed = random.randint(0, 100000)
        task = Task(quadConfig, controlConfig)
        results = task.executeTask()
        # print(str(controller) + " " + str(results))
        # self.saveResult(controlConfig, results)
        return  results

    def saveResult(self, controllerParameters, particlePerformance):
        datapoint = {'ControllerParameters': controllerParameters,
                     'Environment': self.TaskEnv,
                     'performance': particlePerformance}
        self.database.append(datapoint)

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

    def sampleAndEvaluateParticles(self):
        sampleInd = np.random.choice(len(self.particles), self.numberOfParticlesToSample, p=self.weights)

        for ind in sampleInd:
            particle = self.particles[ind]
            # sample from the continoues range around the particle
            # kp = np.random.uniform(max(particle[0] - 500, self.CPFConfig['ParameterRanges']["P"]['min']),
            #                        min(particle[0] + 500, self.CPFConfig['ParameterRanges']["P"]['max']))
            # ki = np.random.uniform(max(particle[1] - 500, self.CPFConfig['ParameterRanges']["I"]['min']),
            #                        min(particle[1] + 500, self.CPFConfig['ParameterRanges']["I"]['max']))
            # kd = np.random.uniform(max(particle[2] - 500, self.CPFConfig['ParameterRanges']["D"]['min']),
            #                        min(particle[2] + 500, self.CPFConfig['ParameterRanges']["D"]['max']))
            kp = particle[0]
            kd = particle[1]

            # Test the particle on predefined trajectory
            # performance = self.evaluateParticle([kp, ki, kd])
            performance = self.evaluateParticle([kp, kd])

            self.performances[ind].append(performance)
        return

    def calculateAverageParticlePerformance(self):
        # compute average performance of each particle
        ind = 0
        n = self.averagePerformanceLength
        for partPerfs in self.performances:
            # print(ind , partPerfs)
            if (len(partPerfs) > n):
                avg = np.average(partPerfs[-n:])
            else:
                avg = np.average(partPerfs)
            self.avgPerformances[ind] = avg
            ind += 1

        self.totalPerformance = sum(self.avgPerformances)
        return

    def updateParticleWeights(self):
        for ind in range(0, len(self.weights)):
            self.weights[ind] = (self.avgPerformances[ind] / self.totalPerformance)

        return

    def checkEndCriteria(self):
        # minEp = 100
        # if (len(volumes) > minEp):
        #     avgClusterDerivitive = np.gradient(self.clusterNumRecord[-self.CPFConfig['averagePerformanceLength']:])
        #     print("volume der. : " + str(avgClusterDerivitive))
        #
        #     Tuned = all(i <= 0.05 for i in avgClusterDerivitive)
        #     print()

        if self.numberRuns < self.MaxIter:
            self.numberRuns += 1
        else:
            self.finished = True



        return

    def clusterParticles(self):
        # min_sample = 2 * 2 #len(self.ParameterRanges)  # 2 times the dimensionality of the data (PID)
        # # min_sample = 1  # 2 times the dimensionality of the data (PID)
        # self.min_sample

        # avgMaxParameterRange = np.average( [ self.ParameterRanges[param].max for param in self.ParameterRanges ])
        # #set epsilon to slightly larger than the distance between particles
        # self.epsilon = (avgMaxParameterRange/self.NumberParticlesPerDimension)*1.5
        # epsilon = 1300


        #remove bad performing particles
        index = 0
        goodParticles = []
        for particle in self.particles:
            # print(self.avgPerformances[index])
            if self.avgPerformances[index] > self.performanceThreshold:
            # if self.avgPerformances[index] >= self.CPFConfig['StartingPerformance']:
                goodParticles.append(particle)
            index += 1
        # print(len(goodParticles))
        if (len(goodParticles) > self.min_sample):
            dbClusters = DBSCAN(eps=self.epsilon, min_samples=self.min_sample).fit(goodParticles)
            labels = dbClusters.labels_
            # print(labels)
            numClusters = max(labels) + 1
            if numClusters == 0:
                # print("no clusters found")
                self.clusterPoints = []
                return []
            else:
                # print("Found Clusters: ")
                # print(max(labels) + 1)
                largestClusterIndex = None
                largestClusterSize = 0
                for clusterIndex in set(labels):
                    if clusterIndex > -1:
                        clusterSize = list(labels).count(clusterIndex)
                        if clusterSize > largestClusterSize:
                            largestClusterIndex = clusterIndex
                            largestClusterSize = clusterSize
                counter = 0
                ClusterIndexes= []
                ClusterParticles = []
                for cindex in labels:
                    if cindex == largestClusterIndex:
                        ClusterIndexes.append(counter)
                        ClusterParticles.append(goodParticles[counter])
                    counter+=1
                #
                # ClusterIndexes = list(filter(lambda x : x == largestClusterIndex, labels))
                # # ClusterParticles = []
                # for ind in ClusterIndexes:
                #     print(ind)
                #     ClusterParticles.append(self.particles[ind])
                # print("Good Particles ")
                # print(goodParticles)
                # print("ClusterIndexes of largest cluster")
                # print(ClusterIndexes)
                # print("Particles of largest cluster")
                # print(ClusterParticles)
                self.clusterPoints = ClusterParticles
                hull = ConvexHull(self.clusterPoints)
                self.currentHull = hull

                return ClusterParticles
        return []

    def calculateClusterAndHull(self):
        if not self.currentHull == []:
            # print(self.clusterPoints[self.currentHull.vertices,0], self.clusterPoints[self.currentHull.vertices,1])
            hull_indices = np.array(self.currentHull.vertices)
            hull_pts = []
            for ind in hull_indices:
                hull_pts.append(self.clusterPoints[ind])
            print(hull_pts)
            self.IntermediateHulls.append(hull_pts)
        else:
            print("No Hull Found")
        return


    #     cluster = self.clusterParticles()
    #     clusterPoints = np.array(cluster)
    #     if not cluster == []:
    #         try:
    #             self.currentHull = ConvexHull(clusterPoints)
    #
    #         except:
    #             print("Hull failed")
    #             hull = []

        # return

    def getCombinedHull(self):
        # print(self.IntermediateHulls)

        allpoints = []
        for hull in self.IntermediateHulls:
            for point in hull:
                allpoints.append(point)


        if not allpoints == []:
            hull = ConvexHull(allpoints)
            hull_indices = np.array(hull.vertices)
            hull_pts = []
            for ind in hull_indices:
                hull_pts.append(allpoints[ind])
            print("Final Result =" + str(hull_pts))
            self.clusterPoints = allpoints
            self.currentHull = hull
            return hull_pts
        else:
            print("Failed combined hull")
            return []


    def render3D(self):
        self.renderCount +=1
        if self.render:
            self.ax.cla()

            # self.ax1.cla()
            # self.clusterPoints = [(1000,2000),(2000,2000),(2000,1000), (4000,4000)]
            # self.currentHull = ConvexHull(self.clusterPoints)
            clusterPoints = np.array(self.clusterPoints)
            ClusterS = [ 50 for a in clusterPoints ]
            ClusterC = [ 1 for a in clusterPoints ]
            # print(clusterPoints)
            ind = 0

            tileSize =1000/self.NumberParticlesPerDimension
            particles= []
            xdata= []
            ydata= []
            particleS = []
            particleC = [[]]
            row=0
            threshold=[[]]
            for entry in self.particles:
                # if self.avgPerformances[ind] > -:
                particles.append(entry)
                xdata.append(entry[0])
                ydata.append(entry[1])
                # zdata.append(entry[2])
                # print(self.avgPerformances[ind])

                if len(particleC[row]) == self.NumberParticlesPerDimension :
                    particleC.append([])
                    threshold.append([])
                    row +=1
                particleC[row].append(3000 + max(self.avgPerformances[ind], -3000) )
                threshold[row].append( 3000 - abs(self.performanceThreshold))
                if (self.avgPerformances[ind] >= self.performanceThreshold ):
                    particleS.append(tileSize*3)
                else:
                    particleS.append(tileSize)

                ind += 1

            # x = np.arange(self.ParameterRanges.P.min, self.ParameterRanges.P.max,
            #               ((self.ParameterRanges.P.max - self.ParameterRanges.P.min)/self.NumberParticlesPerDimension))
            # y = np.arange(self.ParameterRanges.D.min, self.ParameterRanges.D.max,
            #               ((self.ParameterRanges.D.max - self.ParameterRanges.D.min) / self.NumberParticlesPerDimension))

            # self.ax1.scatter(particles.T[0], particles.T[1], particles.T[2], alpha=0.01, s=particleS, c=particleC, cmap="Set1")
            p = self.ax.scatter(xdata,ydata, particleC,
                            s=particleS,
                            alpha=0.5 ,
                            c=particleC,
                            cmap='PiYG',
                            # marker='s' ,
                            edgecolor="k",
                            vmin=0,
                            vmax=3000)

            if (len(clusterPoints) > 0):
                # self.ax.scatter(clusterPoints.T[0], clusterPoints.T[1], clusterPints.T[2], alpha=0.01 , s=ClusterS, c=ClusterC, cmap="Set1")
                self.ax.scatter(clusterPoints.T[0], clusterPoints.T[1], alpha=1 , marker="s", s=ClusterS, edgecolor="k")

                for s in self.currentHull.simplices:
                    s = np.append(s, s[0])  # Here we cycle back to the first coordinate
                    self.ax.plot(clusterPoints[s, 0], clusterPoints[s, 1], "k-", linewidth=3)

            # for hull in self.IntermediateHulls:
            #     for s in hull.simplices:
            #         s = np.append(s, s[0])  # Here we cycle back to the first coordinate
            #         self.ax.plot(clusterPoints[s, 0], clusterPoints[s, 1], "k-")
            # =======


            # p = self.ax.pcolormesh(X,
            #                      Y,
            #                      Z,
            #                     cmap='PiYG',
            #                    linewidth=0.5,
            #                    vmin=0,
            #                    vmax=3000)
            if self.cbar:
                self.cbar.remove()
            self.cbar = plt.colorbar(p)


            self.ax.set_title("Clustered Particle Filtering -" + str(self.title))
            # self.ax.set_xlim(self.ParameterRanges.P.min,
            #                  self.ParameterRanges.P.max)
            #
            # self.ax.set_ylim(self.CPFConfig['ParameterRanges']["I"]['min'], self.CPFConfig['ParameterRanges']["I"]['max'])
            self.ax.set_ylim(self.ParameterRanges.D.min,self.ParameterRanges.D.max)
            self.ax.set_xlabel("P-Gain")
            self.ax.set_ylabel("D-Gain")
            # self.ax.set_zlabel("Performance")
            # self.ax.legend()

            plt.draw()

            plt.pause(0.0001)
            plt.savefig("CPF-Images/" + str(self.renderCount) + ".png")
            # time.sleep(10)
        return

    def runParticleFiltering(self):
        while not self.finished:

            self.setRandomEnvironmentMagnitude()

            self.sampleAndEvaluateParticles()

            self.calculateAverageParticlePerformance()

            self.updateParticleWeights()

            self.checkEndCriteria()

            self.clusterParticles()

            if self.render:
                self.render3D()
        return

    def run(self):
        print("CPF running")
        self.renderCount = 0
        self.title = ""
        EnvCount = 0
        Envs = []
        for env in self.Task.Env:
            if self.Task.Env[env].enabled:
                Envs.append(env)


        if Envs == []:
            Envs.append(None)

        for env in Envs:
            print( "Running " + env)
            self.title = env
            EnvCount +=1
            self.initilizeParticles()

            if self.render:
                self.render3D()

            self.setTaskEnv(env)

            self.runParticleFiltering()

            self.calculateClusterAndHull()


            self.currentHull = []



        ControllerParameterSet = self.getCombinedHull()
        if self.render:
            self.render3D()
        time.sleep(10)
        return ControllerParameterSet


def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()
    config = CONFIG_FACTORY.merge()
    random.seed(config.random_seed)
    results = []
    for i in range(config.numberIterations):
        algo = ClusteredParticleFilteringOptimization(**config.CPF)
        result = algo.run()
        results.append(result)



    # p_average = 0
    # # i_average = 0
    # d_average = 0
    #
    # for result in results:
    #     p = result[0][0]
    #     d = result[0][1]
    #     # d = result[0][2]
    #     p_average+=p
    #     d_average+=d
    #     # d_average+=d
    #
    # p_average = p_average/len(results)
    # d_average = d_average/len(results)
    # # d_average = d_average/len(results)
    #
    # print(p_average)
    # print(d_average)


    return



if __name__ == "__main__":
    main()
