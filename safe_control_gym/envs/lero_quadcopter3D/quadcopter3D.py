import random
import scipy.integrate
import datetime
import signal
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import sys
import math
import copy




QUADCOPTER_PARAMETERS =  {
            "position": [0, 0, 0],
            "orientation": [0, 0, 0],
            "L": 0.3,
            "r": 0.1,
            "prop_size": [10, 4.5],
            "weight": 1.2
}
DefaultEnv = {
    "RotorFault":{
          "enabled": False,
          "min_magnitude" : 0,
          "max_magnitude" : 0.30,
          "randomTime" : True,
          "starttime": 500,
          "endtime" : 31000,
          "randomRotor" : True,
          "faultRotorID" : 1
    },
    "Wind":{
          "enabled": False,
          "min_magnitude" : 0,
          "max_magnitude" : 3,
          "randomDirection" : False,
          "direction" : 1
      },
      "PositionNoise": {
          "enabled": False,
          "min_magnitude": 0,
          "max_magnitude" : 0.3
      },
      "AttitudeNoise": {
          "enabled": False,
          "min_magnitude" : 0,
          "max_magnitude" : 0.1
      }
    }

DefaultPath = {
    "randomPath" : False,
    "randomSeed" : 1234,
    "randomLimit": 8,
    "waypoints":{
        "x": [0, 0],
        "y": [0, 0],
        "z": [0, 5]
    },
    "safetyRadius": 1,
    "maxStepsPerRun": 3000,
    "stablilizationAtGoal": 100

}

# Low altitude Model
# transfer function for along-wind
def u_transfer_function(height, airspeed):
    # turbulence level defines value of wind speed in knots at 20 feet
    # turbulence_level = 15 * 0.514444 # convert speed from knots to meters per second
    turbulence_level = 15
    length_u = height / ((0.177 + 0.00823 * height) ** (0.2))
    # length_u = 1750
    sigma_w = 0.1 * turbulence_level
    sigma_u = sigma_w / ((0.177 + 0.000823 * height) ** (0.4))
    num_u = [sigma_u * (math.sqrt((2 * length_u) / (math.pi * airspeed))) * airspeed]
    den_u = [length_u, airspeed]
    H_u = signal.TransferFunction(num_u, den_u)
    return H_u


# transfer function for cross-wind
def v_transfer_function(height, airspeed):
    # turbulence level defines value of wind speed in knots at 20 feet
    # turbulence_level = 15 * 0.514444 # convert speed from knots to meters per second
    turbulence_level = 15
    length_v = height / ((0.177 + 0.00823 * height) ** (0.2))
    # length_v = 1750
    sigma_w = 0.1 * turbulence_level
    sigma_v = sigma_w / ((0.177 + 0.000823 * height) ** (0.4))
    b = sigma_v * (math.sqrt((length_v) / (math.pi * airspeed)))
    Lv_V = length_v / airspeed
    num_v = [(math.sqrt(3) * Lv_V * b), b]
    den_v = [(Lv_V ** 2), 2 * Lv_V, 1]
    H_v = signal.TransferFunction(num_v, den_v)
    return H_v


# transfer function for vertical-wind
def w_transfer_function(height, airspeed):
    # turbulence level defines value of wind speed in knots at 20 feet
    # turbulence_level = 15 * 0.514444 # convert speed from knots to meters per second
    turbulence_level = 15
    length_w = height
    # length_w = 1750
    sigma_w = 0.1 * turbulence_level
    c = sigma_w * (math.sqrt((length_w) / (math.pi * airspeed)))
    Lw_V = length_w / airspeed
    num_w = [(math.sqrt(3) * Lw_V * c), c]
    den_w = [(Lw_V ** 2), 2 * Lw_V, 1]
    H_v = signal.TransferFunction(num_w, den_w)
    return H_v

class GUI():

    def __init__(self, quads, path , safebound, radius=1):
        self.quads = quads
        self.pos = []
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        self.set_plot_bounds()
        self.init_plot()
        self.currentPoints = []
        self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)
        self.scatter = 0
        self.safe_bound = safebound
        self.path = path
        self.radius = radius
        self.min_distances_points = []
        self.min_distances = []
        plt.ion()
        plt.show()





    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def init_plot(self):

        self.quads['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
        self.quads['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
        # self.quads['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)

    def set_plot_bounds(self):
        self.bounds = 5
        self.ax.set_xlim3d([-self.bounds, self.bounds])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-self.bounds, self.bounds])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, self.bounds])
        self.ax.set_zlabel('Z')
        self.ax.set_title('3D Quadcopter Simulation')


    def update(self, quad, bound):
        self.safe_bound = bound
        self.quads = quad
        self.ax.cla()
        self.init_plot()
        self.set_plot_bounds()
        R = self.rotation_matrix(self.quads['orientation'])
        L = self.quads['L']
        points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
        points = np.dot(R,points)
        points[0,:] += self.quads['state'][0]
        points[1,:] += self.quads['state'][1]
        points[2,:] += self.quads['state'][2]
        self.quads['l1'].set_data(points[0,0:2],points[1,0:2])
        self.quads['l1'].set_3d_properties(points[2,0:2])
        self.quads['l2'].set_data(points[0,2:4],points[1,2:4])
        self.quads['l2'].set_3d_properties(points[2,2:4])
        # self.quads['hub'].set_data(points[0,5],points[1,5])
        # self.quads['hub'].set_3d_properties(points[2,5])
        pos = [self.quads['state'][0],
               self.quads['state'][1],
               self.quads['state'][2]]
        self.pos.append(pos)




        self.updateQuadLine()
        self.showWaypoints()
        self.updatePathToGoal()
        self.showDistanceToRef()

        plt.pause(0.0001)


    def showDistanceToRef(self):

        #get closest point on the linspace between waypoints

        p1 =[self.pos[-1][0],self.pos[-1][1],self.pos[-1][2]]
        distances = []
        points = []
        for i in range(len(self.safe_bound)):

            p2 = np.array([self.safe_bound[i][0], self.safe_bound[i][1], self.safe_bound[i][2]])
            squared_dist = np.sum((p1 - p2) ** 2, axis=0)
            dist = np.sqrt(squared_dist)
            distances.append(dist)
            points.append(p2)
            #print(str(self.safe_bound[i]) +" "+ str(dist))

        i = np.where(distances == np.amin(distances))
        index = i[0][0]

        self.min_distances_points.append((p1, points[index]) )
        self.min_distances.append(distances[index])
        ind = 0
        for (quadPoint,minDistancePoint) in self.min_distances_points:
            xs = [quadPoint[0], minDistancePoint[0]]
            ys = [quadPoint[1], minDistancePoint[1]]
            zs = [quadPoint[2], minDistancePoint[2]]
            if self.min_distances[ind] <= self.radius:
                color = "g"
            else:
                color = "r"
            ind+=1
            self.ax.plot3D(xs,ys,zs,linewidth=1, alpha=0.5, c=color)

        return


    def updateQuadLine(self):
        x_c1 = []
        y_c1 = []
        z_c1 = []

        for i in range(len(self.pos)):
            x_c1.append(self.pos[i][0])
            y_c1.append(self.pos[i][1])
            z_c1.append(self.pos[i][2])

        self.ax.plot3D(x_c1,y_c1,z_c1,linewidth=2, c="b")


    def showWaypoints(self):
        xs = self.path["x"]
        ys = self.path["y"]
        zs = self.path["z"]
        for i in range(len(xs)):
            self.ax.scatter(xs[i], ys[i], zs[i], marker='o', color='k', alpha=0.5, linewidth=5)

    def updatePathToGoal(self):

        xs = self.path['x']
        ys = self.path['y']
        zs = self.path['z']

        self.ax.plot3D(xs, ys, zs, linewidth=1, c="k")

        # using scatter plot does not accurately show the size of the safebound- its a crude estimate.
        # path = self.safe_bound
        # ind = 0
        # width = 20 * self.radius
        # for i in range(len(path)):
        #     pos = path[i]
        #     ind += 1
        #     self.ax.scatter(pos[0], pos[1], pos[2], marker='o',color='green', alpha=0.05, linewidth=width)


    def keypress_routine(self,event):
        sys.stdout.flush()
        if event.key == 'x':
            y = list(self.ax.get_ylim3d())
            y[0] += 0.2
            y[1] += 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'w':
            y = list(self.ax.get_ylim3d())
            y[0] -= 0.2
            y[1] -= 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'd':
            x = list(self.ax.get_xlim3d())
            x[0] += 0.2
            x[1] += 0.2
            self.ax.set_xlim3d(x)
        elif event.key == 'a':
            x = list(self.ax.get_xlim3d())
            x[0] -= 0.2
            x[1] -= 0.2
            self.ax.set_xlim3d(x)

    def close(self):

        plt.close(self.fig)

class Propeller():
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.thrust_unit = thrust_unit
        self.speed = 0 #RPM
        self.thrust = 0
        self.fault_mag = 0

    def set_speed(self,speed):
        self.speed = speed
        if self.fault_mag > 0:
            # print(self.fault_mag)
            # print("Speed before :" + str(self.speed))
            self.speed = self.speed * (1 - self.fault_mag)
            # print("Speed after :" + str(self.speed))
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))
        self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)

        if self.thrust_unit == 'Kg':
            self.thrust = self.thrust*0.101972
    def get_speed(self):
        return self.speed

    def set_fault(self,fault):
        self.fault_mag = fault


class Quadcopter():
    # State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
    # From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
    def __init__(self,
                 Quad = QUADCOPTER_PARAMETERS ,
                 gravity=9.81,
                 b=0.0245,
                 Env = DefaultEnv,
                 Path = DefaultPath,
                 render=False
                 ):

        self.CTRL_TIMESTEP = 500
        self.quads = Quad
        self.g = gravity
        self.b = b

        self.Path = Path

        if self.Path['randomPath'] == 'True' :
            self.setRandomPath()
        else:
            self.setPath()
        self.total_time_outside_safety = 0
        self.done = False
        self.Env = Env
        self.faultModes = []
        self.setEnv()

        self.ode =  scipy.integrate.ode(self.state_dot).set_integrator('vode',nsteps=self.CTRL_TIMESTEP,method='bdf')
        self.time = datetime.datetime.now()

        self.stepNum = 0
        self.quads['state'] = np.zeros(12)
        self.quads['state'][0:3] = self.quads['position']
        self.quads['state'][6:9] = self.quads['orientation']
        self.quads['m1'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
        self.quads['m2'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
        self.quads['m3'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
        self.quads['m4'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
        # From Quadrotor Dynamics and Control by Randal Beard
        ixx=((2*self.quads['weight']*self.quads['r']**2)/5)+(2*self.quads['weight']*self.quads['L']**2)
        iyy=ixx
        izz=((2*self.quads['weight']*self.quads['r']**2)/5)+(4*self.quads['weight']*self.quads['L']**2)
        self.quads['I'] = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
        self.quads['invI'] = np.linalg.inv(self.quads['I'])

        self.rend = render
        if self.rend: self.setupGUI()


    def setupGUI(self):

        self.GUI = GUI(self.quads, self.Path['waypoints'] , self.safe_bound, self.Path['safetyRadius'])

        return

#==========Path related functions =========================

    def setRandomPath(self):
        self.stepsToGoal = 0
        self.goals = []
        self.safe_region = []
        self.trackingAccuracy = self.Path['safetyRadius']
        self.maxSteps = self.Path['maxStepsPerRun']
        self.requiredStableAtGoal = self.Path['stablilizationAtGoal']
        limit = self.Path['randomLimit']
        # np.random.seed(self.Path['randomSeed'])
        x_dest = np.random.randint(-limit, limit)
        y_dest = np.random.randint(-limit, limit)
        z_dest = np.random.randint(5, limit)
        x_dest2 = np.random.randint(-limit, limit)
        y_dest2 = np.random.randint(-limit, limit)
        z_dest2 = np.random.randint(5, limit)
        x_path = [0, 0, x_dest, x_dest2]
        y_path = [0, 0, y_dest, y_dest2]
        z_path = [0, 5, z_dest, z_dest2]
        steps = len(x_path)
        interval_steps = 50
        self.goals = []
        self.safe_region = []
        self.stepsToGoal=0
        for i in range(steps):
            if (i < steps - 1):
                # create linespace between waypoint i and i+1
                x_lin = np.linspace(x_path[i], x_path[i + 1], interval_steps)
                y_lin = np.linspace(y_path[i], y_path[i + 1], interval_steps)
                z_lin = np.linspace(z_path[i], z_path[i + 1], interval_steps)
            else:
                x_lin = np.linspace(x_path[i], x_path[i], interval_steps)
                y_lin = np.linspace(y_path[i], y_path[i], interval_steps)
                z_lin = np.linspace(z_path[i], z_path[i], interval_steps)

            self.goals.append([x_path[i], y_path[i], z_path[i]])
            # for each pos in linespace append a goal
            self.safe_region.append([])
            for j in range(interval_steps):
                self.safe_region[i].append([x_lin[j], y_lin[j], z_lin[j]])
                self.stepsToGoal += 1
        self.currentWaypoint = 0
        self.numberOfWaypoints =steps
        self.updateWaypoint(self.currentWaypoint)
        self.trackingAccuracy = self.Path['safetyRadius']
        self.maxSteps = self.Path['maxStepsPerRun']
        self.requiredStableAtGoal = self.Path['stablilizationAtGoal']

        self.Path['waypoints'] = {
            "x": x_path,
            "y": y_path,
            "z": z_path
        }
        return

    def setPath(self):

        self.stepsToGoal = 0
        self.goals = []
        self.safe_region = []
        self.trackingAccuracy = self.Path['safetyRadius']
        self.maxSteps = self.Path['maxStepsPerRun']
        self.requiredStableAtGoal = self.Path['stablilizationAtGoal']
        x_path = self.Path['waypoints']['x']
        y_path = self.Path['waypoints']['y']
        z_path = self.Path['waypoints']['z']
        steps = len(x_path)
        interval_steps = 50

        for i in range(steps):
            if (i < steps - 1):
                # create linespace between waypoint i and i+1
                x_lin = np.linspace(x_path[i], x_path[i + 1], interval_steps)
                y_lin = np.linspace(y_path[i], y_path[i + 1], interval_steps)
                z_lin = np.linspace(z_path[i], z_path[i + 1], interval_steps)
            else:
                x_lin = np.linspace(x_path[i], x_path[i], interval_steps)
                y_lin = np.linspace(y_path[i], y_path[i], interval_steps)
                z_lin = np.linspace(z_path[i], z_path[i], interval_steps)

            self.goals.append([x_path[i], y_path[i], z_path[i]])
            # for each pos in linespace append a goal
            self.safe_region.append([])
            for j in range(interval_steps):
                self.safe_region[i].append([x_lin[j], y_lin[j], z_lin[j]])
                self.stepsToGoal += 1

        self.numberOfWaypoints = steps
        self.currentWaypoint = 0
        self.updateWaypoint(self.currentWaypoint)
        return

    def isAtPosition(self,pos):
        [dest_x, dest_y, dest_z] = pos
        [x, y, z ] = self.get_position()
        x_error = dest_x - x
        y_error = dest_y - y
        z_error = dest_z - z
        total_distance_to_goal = abs(x_error) + abs(y_error) + abs(z_error)
        isAt = True if total_distance_to_goal < self.trackingAccuracy else False
        return isAt

    def updateWaypoint(self, goalIndex):

        self.target = self.goals[goalIndex]
        self.safe_bound = self.safe_region[max(goalIndex-1,0)]

        return

    def getMinDistanceToReference(self):
        #get closest point on the line space between waypoints
        p1 = self.get_position()
        distances = []
        points = []
        for i in range(len(self.safe_bound)):
            p2 = np.array([self.safe_bound[i][0], self.safe_bound[i][1], self.safe_bound[i][2]])
            squared_dist = np.sum((p1 - p2) ** 2, axis=0)
            dist = np.sqrt(squared_dist)
            distances.append(dist)
            points.append(p2)
            #print(str(self.safe_bound[i]) +" "+ str(dist))
        i = np.where(distances == np.amin(distances))
        index = i[0][0]
        return distances[index]

    def checkSafetyBound(self):
        self.current_distance_to_opt = self.getMinDistanceToReference()
        if  self.current_distance_to_opt > self.trackingAccuracy :
            self.total_time_outside_safety += 1
        return


#==========Environment related functions =========================
    def setEnv(self):
        if self.Env == []:
            faultModes = ['None']
        # ===============Rotor Fault config====================
        keys = self.Env.keys()

        if "RotorFault" in keys and self.Env['RotorFault']['enabled']:
            self.setupRotorFaults()
        # ===============Wind gust config=================
        if "Wind" in keys and self.Env['Wind']['enabled'] :
            self.setupWind()
        #============= Position Noise config===============
        if "PositionNoise" in keys and self.Env['PositionNoise']['enabled']:
            self.setupPositionSensorNoise()
        # ============= Attitude Noise config===============
        if "AttitudeNoise" in keys and self.Env['AttitudeNoise']['enabled']:
            self.setupAttitudeSensorNoise()

        return

    def setupRotorFaults(self):
        self.faultModes.append("RotorFault")
        fault_mag = self.Env['RotorFault']['magnitude']
        faults = [0, 0, 0, 0]
        if bool(self.Env['RotorFault']['randomRotor']):
            rotor = np.random.randint(0, 4)
        else:
            rotor = self.Env['RotorFault']['faultRotorID']
        faults[rotor] = fault_mag
        self.rotorFault = faults
        if bool(self.Env['RotorFault']['randomTime']):
            # randomly trigger in first half of simulation
            stime = random.randint(10, int(self.maxSteps / 2))
            etime = random.randint(int(self.maxSteps / 2), self.maxSteps)
            self.rotorFaultStart = stime
            self.rotorFaultEnd = etime
        else:
            self.rotorFaultStart = self.Env['RotorFault']['starttime']
            self.rotorFaultEnd = self.Env['RotorFault']['endtime']

        return

    def set_motor_faults(self, faults):
        f1 = faults[0]
        f2 = faults[1]
        f3 = faults[2]
        f4 = faults[3]
        self.quads['m1'].set_fault(f1)
        self.quads['m2'].set_fault(f2)
        self.quads['m3'].set_fault(f3)
        self.quads['m4'].set_fault(f4)

        return

    def setupPositionSensorNoise(self):
        self.faultModes.append("PositionNoise")
        noise =  self.Env['PositionNoise']['magnitude']
        self.posNoiseMag = noise

    def setupAttitudeSensorNoise(self):
        self.faultModes.append("AttitudeNoise")
        noise = self.Env['AttitudeNoise']['magnitude']
        self.attNoiseMag = noise

    def setupWind(self):
        self.faultModes.append("Wind")
        if bool(self.Env['Wind']['randomDirection']):
            direction = np.random.randint(0, 4)
        else:
            direction = self.Env['Wind']['direction']

        WindMag = self.Env['Wind']['magnitude']
        if (direction == 0):
            winds = [-WindMag, 0, 0]
        elif (direction == 1):
            winds = [WindMag, 0, 0]
        elif (direction == 2):
            winds = [0, -WindMag, 0]
        else:
            winds = [0, WindMag, 0]



        self.windMag = WindMag
        self.airspeed = 15 # default airspeed
        self.randWind = self.generate_wind_turbulence(5)
        self.XWind = 0
        self.YWind = 0
        self.ZWind = 0
        self.setNormalWind(winds)

    def setWind(self, wind_vec):
        self.randWind = wind_vec

    def setNormalWind(self, winds):
        self.XWind = winds[0]
        self.YWind = winds[1]
        self.ZWind = winds[2]

    def generate_wind_turbulence(self, h):

        height = float(h) * 3.28084
        airspeed = float(self.airspeed) * 3.28084

        mean = 0
        std = 1
        # create a sequence of 1000 equally spaced numeric values from 0 - 5
        num_samples = self.maxSteps
        t_p = np.linspace(0, 10, num_samples)
        samples1 = 10 * np.random.normal(mean, std, size=num_samples)
        samples2 = 10 * np.random.normal(mean, std, size=num_samples)
        samples3 = 10 * np.random.normal(mean, std, size=num_samples)

        tf_u = u_transfer_function(height, airspeed)
        tf_v = v_transfer_function(height, airspeed)
        tf_w = w_transfer_function(height, airspeed)

        tout1, y1, x1 = signal.lsim(tf_u, samples1, t_p)
        # tout1, y1, x1 = signal.lsim(tf_u, n1, t_w)
        # covert obtained values to meters/second
        y1_f = [i * 0.305 for i in y1]
        tout2, y2, x2 = signal.lsim(tf_v, samples2, t_p)
        # tout2, y2, x2 = signal.lsim(tf_v, n2, t_w)
        y2_f = [i * 0.305 for i in y2]
        tout3, y3, x3 = signal.lsim(tf_w, samples3, t_p)
        # tout3, y3, x3 = signal.lsim(tf_w, n3, t_w)
        y3_f = [i * 0.305 for i in y3]

        return [y1_f, y2_f, y3_f]

#===============Simulation Functions ===========

    def update(self, dt=0.02):
        self.stepNum += 1

        # Handle Rotor Fault Here
        if "RotorFault" in  self.faultModes:
            if self.stepNum > self.rotorFaultStart:
                self.set_motor_faults(self.rotorFault)
            if self.stepNum > self.rotorFaultEnd:
                self.set_motor_faults([0, 0, 0, 0])

        self.ode.set_initial_value(self.quads['state'],
                                   0).set_f_params()
        ## removed key from multi drone object - might cause priblems ( .set_f_params(key))
        self.quads['state'] = self.ode.integrate(self.ode.t + dt)
        self.quads['state'][6:9] = self.wrap_angle(self.quads['state'][6:9])
        self.quads['state'][2] = max(0, self.quads['state'][2])

        #check if the quad reached the next waypoint or finished
        self.updatePathPosition()

    def state_dot(self, time, state):
        state_dot = np.zeros(12)
        # The velocities(t+1 x_dots equal the t x_dots)
        state_dot[0] = self.quads['state'][3]
        state_dot[1] = self.quads['state'][4]
        state_dot[2] = self.quads['state'][5]
        # The acceleration
        height = self.quads['state'][2]
        #
        F_d  = np.array([ 0, 0, 0])
        #
        air_density = 1.225 #kg/m^3
        C_d = 1
        cube_width = 0.1 # 10cm x 10cm cube as shape model of quadcopter
        A_yz = cube_width*cube_width
        A_xz = cube_width*cube_width
        A_xy = cube_width*cube_width

        A = [ A_yz , A_xz  , A_xy  ] # cross sectional area in each axis perpendicular to velocity axis

        #if wind is active the velocity in each axis is subject to wind
        if "Wind" in self.faultModes:
            nomX = self.XWind
            nomY = self.YWind
            nomZ = self.ZWind
            randX = self.randWind[0][self.stepNum]
            randY = self.randWind[1][self.stepNum]
            randZ = self.randWind[2][self.stepNum]
            wind_velocity_vector = [ nomX + randX  , nomY + randY  , nomZ + randZ] # wind velocity in each axis
        else:
            wind_velocity_vector = [0 ,0 ,0]

        wind_vel_inertial_frame = np.dot(self.rotation_matrix(self.quads['state'][6:9]) ,  wind_velocity_vector)
        V_b = [ state[0], state[1] , state[2]]
        V_a = wind_vel_inertial_frame  - V_b
        DragVector = [
            A[0] * (V_a[0] * abs(V_a[0])),
            A[1] * (V_a[1] * abs(V_a[1])),
            A[2] * (V_a[2] * abs(V_a[2]))
        ]
        F_d = [i * (0.5 * air_density * C_d) for i in DragVector]
        #Dryden turbulence random component
            # self.windMag

        x_dotdot = np.array([0,0,-self.quads['weight']*self.g]) \
                   + np.dot(self.rotation_matrix(self.quads['state'][6:9]),
                            np.array([0,0,(self.quads['m1'].thrust +
                                           self.quads['m2'].thrust +
                                           self.quads['m3'].thrust +
                                           self.quads['m4'].thrust)])) \
                   /self.quads['weight'] + F_d

        state_dot[3] = x_dotdot[0]
        state_dot[4] = x_dotdot[1]
        state_dot[5] = x_dotdot[2]
        # The angular rates(t+1 theta_dots equal the t theta_dots)
        state_dot[6] = self.quads['state'][9]
        state_dot[7] = self.quads['state'][10]
        state_dot[8] = self.quads['state'][11]
        # The angular accelerations
        omega = self.quads['state'][9:12]
        tau = np.array([self.quads['L']*
                        (self.quads['m1'].thrust-self.quads['m3'].thrust),
                         self.quads['L']*(self.quads['m2'].thrust-self.quads['m4'].thrust),
                         self.b*(self.quads['m1'].thrust-self.quads['m2'].thrust +
                                 self.quads['m3'].thrust-self.quads['m4'].thrust)])
        omega_dot = np.dot(self.quads['invI'], (tau - np.cross(omega, np.dot(self.quads['I'],omega))))
        state_dot[9] = omega_dot[0]
        state_dot[10] = omega_dot[1]
        state_dot[11] = omega_dot[2]
        return state_dot

    def updatePathPosition(self):

        if self.stepNum > self.maxSteps:
            self.done = True

        if (self.isAtPosition(self.goals[self.currentWaypoint])):

            # if there is another waypoint then set the quadcopters next goal and safety margin
            if (self.currentWaypoint < self.numberOfWaypoints - 1):
                self.currentWaypoint += 1
                self.updateWaypoint(self.currentWaypoint)
            else:
                # there is no more waypoints in the trajectory
                self.stableAtGoal += 1
                if (self.stableAtGoal > self.requiredStableAtGoal):
                    # wait for quadcopter to stabilize around goal
                    self.done = True
                else:
                    self.done = False
        else:
            #the quadcopter has not reached the goal yet
            self.stableAtGoal = 0

        self.checkSafetyBound()


# ==== internal sim and helper functions ========

    def rotation_matrix(self, angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_z = np.array([[cg, -sg, 0], [sg, cg, 0], [0, 0, 1]])
        R = np.dot(R_z, np.dot(R_y, R_x))
        return R

    def wrap_angle(self, val):
        return ((val + np.pi) % (2 * np.pi) - np.pi)

    def set_motor_speeds(self,speeds):

        self.quads['m1'].set_speed(speeds[0])
        self.quads['m2'].set_speed(speeds[1])
        self.quads['m3'].set_speed(speeds[2])
        self.quads['m4'].set_speed(speeds[3])

    def get_motor_speeds(self):

        return [self.quads['m1'].get_speed(), self.quads['m2'].get_speed(),
                self.quads['m3'].get_speed(), self.quads['m4'].get_speed()]

    def get_motor_speeds_rpm(self):
        return [self.quads['m1'].get_speed(), self.quads['m2'].get_speed(),self.quads['m3'].get_speed(),self.quads['m4'].get_speed()]

    def get_position(self):
        return self.quads['state'][0:3]

    def get_linear_rate(self):
        return self.quads['state'][3:6]

    def get_orientation(self):
        return self.quads['state'][6:9]

    def get_angular_rate(self):
        return self.quads['state'][9:12]

    def get_state(self):
        return self.quads['state']

    def set_position(self,position):
        self.quads['state'][0:3] = position

    def set_orientation(self,orientation):
        self.quads['state'][6:9] = orientation

    def get_time(self):
        return self.time

    def get_target(self):
        #needed for controllers - next reference point
        return self.target

# ========Safe Control Repo Interface Functions ==========

    def reset(self):

        if self.Path['randomPath']:
            self.setRandomPath()
        else:
            self.setPath()

        self.setEnv()


        self.total_time_outside_safety = 0
        self.done = False
        self.time = datetime.datetime.now()
        self.stepNum = 0
        self.quads['state'] = np.zeros(12)
        self.quads['state'][0:3] = self.quads['position']
        self.quads['state'][6:9] = self.quads['orientation']
        self.quads['m1'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
        self.quads['m2'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
        self.quads['m3'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
        self.quads['m4'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
        # From Quadrotor Dynamics and Control by Randal Beard
        ixx = ((2 * self.quads['weight'] * self.quads['r'] ** 2) / 5) + (
                    2 * self.quads['weight'] * self.quads['L'] ** 2)
        iyy = ixx
        izz = ((2 * self.quads['weight'] * self.quads['r'] ** 2) / 5) + (
                    4 * self.quads['weight'] * self.quads['L'] ** 2)
        self.quads['I'] = np.array([[ixx, 0, 0], [0, iyy, 0], [0, 0, izz]])
        self.quads['invI'] = np.linalg.inv(self.quads['I'])
        if self.rend:
            self.GUI.close()
            self.setupGUI()


        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    def step(self, action):
        self.set_motor_speeds(action)
        self.update()
        obs = self._get_obs()
        info = self._get_info()
        rew = self._get_rew()
        done = self._get_done()

        if self.rend : self.render()

        return obs, rew, done, info

    def render(self):

        if self.rend: self.GUI.update(self.quads, self.safe_bound)
        if self.done: self.GUI.close()

        return

    def _get_obs(self):
        obs = copy.deepcopy(self.get_state())
        if "PositionNoise" in self.faultModes:
            x_noise = np.random.uniform(-self.posNoiseMag, self.posNoiseMag)
            y_noise = np.random.uniform(-self.posNoiseMag, self.posNoiseMag)
            z_noise = np.random.uniform(-self.posNoiseMag, self.posNoiseMag)
            obs[0] += x_noise
            obs[1] += y_noise
            obs[2] += z_noise
        if "AttitudeNoise" in self.faultModes:
            roll_noise = np.random.uniform(-self.attNoiseMag, self.attNoiseMag)
            pitch_noise = np.random.uniform(-self.attNoiseMag, self.attNoiseMag)
            yaw_noise = np.random.uniform(-self.attNoiseMag, self.attNoiseMag)
            obs[6] += roll_noise
            obs[7] += pitch_noise
            obs[8] += yaw_noise
        obs = np.concatenate((obs, self.target))

        return obs

    def _get_info(self):
        #calculate error - distance to safezone
        info = {"mse": self.getMinDistanceToReference()}
        return info

    def _get_rew(self):
        rew = self.total_time_outside_safety
        return rew

    def _get_done(self):
        # check if quad is at final waypoint
        if (self.stepNum > self.maxSteps):
            self.done = True
        return self.done


    def close(self):
        if self.rend:
            self.GUI.close()
        return
#
#
# if __name__ == "__main__":
#     Q = Quadcopter()
#     done = False
#     while not done:
#         action = [5500,5500,5500,5500]
#         obs, rew, done, info = Q.step(action)
#         Q.render()
#
#         print( obs, rew, done, info)