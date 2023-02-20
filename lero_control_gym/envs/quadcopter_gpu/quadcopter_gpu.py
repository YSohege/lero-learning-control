

#
#
#
# # import cupy as cp
# #
# # import timeit
# # exit()
# # import random
# import scipy.integrate
# import datetime
# import signal
# from scipy import signal
# import numpy as np
# import matplotlib.pyplot as plt
# import mpl_toolkits.mplot3d.axes3d as Axes3D
# import sys
# import math
# import copy
# #
# # device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
# # torch.cuda.set_device(device)
#
#
# class Propeller():
#     def __init__(self, prop_dia, prop_pitch, thrust_unit='N', device="cpu"):
#         self.dia        = torch.tensor([prop_dia], device=device, dtype=dtype)
#         self.pitch      = torch.tensor([prop_pitch],device=device, dtype=dtype)
#         self.speed      = torch.tensor([0.0] , device=device, dtype=dtype)
#         self.thrust     = torch.tensor([0.0] , device=device, dtype=dtype)
#         self.fault_mag  = torch.tensor([0.0] , device=device, dtype=dtype)
#         self.mu         = torch.tensor([0.5], device=device, dtype=dtype)
#         self.sigma      = torch.tensor([0.2], device=device, dtype=dtype)
#         self.a          = torch.tensor([4.392e-8], device=device, dtype=dtype)
#         self.b          = torch.tensor([3.5], device=device, dtype=dtype)
#         self.c          = torch.tensor([4.23e-4], device=device, dtype=dtype)
#         self.d          = torch.tensor([0.0], device=device, dtype=dtype)
#         self.e          = torch.tensor([0.0], device=device, dtype=dtype)
#         self.activeFault= torch.tensor([0.0] , device=device, dtype=dtype)
#         self.kgScale    = torch.tensor([0.101972], device=device, dtype=dtype)
#         self.thrust_unit= thrust_unit
#         self.intermittent_fault = True
#
#
#
#
#     def set_intermittent(self, inter , mu, sigma):
#         self.intermittent_fault = inter
#         self.mu = mu
#         self.sigma = sigma
#
#     def set_speed(self,speed):
#         self.speed = torch.tensor([speed], device=device, dtype=dtype)
#         # if self.fault_mag > 0:
#         #     if self.intermittent_fault:
#         #         torch.normal(mean=self.mu,std=self.sigma, out=self.activeFault)
#         #         if self.activeFault > 0.5:
#         #             self.speed = self.speed * (1 - self.fault_mag)
#         #     else:
#         #         self.speed = self.speed * (1 - self.fault_mag)
#         self.thrust = self.a * self.speed * torch.pow(self.dia, self.b) / (torch.sqrt(self.pitch))
#         self.thrust = self.thrust * (self.c * self.speed * self.pitch)
#
#     def set_speed1(self,speed):
#         self.speed = speed
#         # if self.fault_mag > 0:
#         #     if self.intermittent_fault:
#         #         torch.normal(mean=self.mu,std=self.sigma, out=self.activeFault)
#         #         if self.activeFault > 0.5:
#         #             self.speed = self.speed * (1 - self.fault_mag)
#         #     else:
#         #         self.speed = self.speed * (1 - self.fault_mag)
#
#         self.d = self.dia.pow(self.b).div(self.pitch.sqrt())
#         self.thrust = self.a.mul( (self.speed.mul(self.d)) ).mul( (self.c.mul(self.speed.mul(self.pitch))))
#         print(speed)
#
#     def get_speed(self):
#         return self.speed
#
#     def set_fault(self,fault):
#         self.fault_mag = fault
#
#
# def test(iterations=100000):
#     Dia = torch.tensor([10.0], device=device, dtype=dtype)
#     Pitch = torch.tensor([0.3], device=device, dtype=dtype)
#     P = Propeller(Dia, Pitch, device=device)
#     for i in range(iterations):
#         P.set_speed(i)
#         # print(P.thrust)
#
#     return
#
# def test1(iterations=100000):
#     Dia = torch.tensor([10.0], device=device, dtype=dtype)
#     Pitch = torch.tensor([0.3], device=device, dtype=dtype)
#     P = Propeller(Dia, Pitch, device=device)
#     for i in range(iterations):
#         speed = torch.tensor([i], device=device, dtype=dtype)
#         P.set_speed1(speed)
#         # print(P.thrust)
#
#     return
#
# dtype =torch.float
# iter = 10000
# device="cpu"
# # torch.cuda.set_device(device)
# print(device)
# result = timeit.timeit(stmt='test(iter)', globals=globals(), number=1)
# print(result)
# result = timeit.timeit(stmt='test1(iter)', globals=globals(), number=1)
# print(result)
#
# device="cuda:0"
# torch.cuda.set_device(device)
# print(device)
# result = timeit.timeit(stmt='test(100)', globals=globals(), number=1)
#
# result = timeit.timeit(stmt='test(iter)', globals=globals(), number=1)
# print(result)
# result = timeit.timeit(stmt='test1(iter)', globals=globals(), number=1)
# print(result)
#
#
#
# exit()
# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================


import torch
import timeit

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
#
# device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
# torch.cuda.set_device(device)


class Propeller():
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N', device="cpu"):
        self.dia = torch.tensor([prop_dia], device=device, dtype=dtype)
        self.pitch = torch.tensor([prop_pitch], device=device, dtype=dtype)
        self.speed = torch.tensor([0.0], device=device, dtype=dtype)
        self.thrust = torch.tensor([0.0], device=device, dtype=dtype)
        self.fault_mag = torch.tensor([0.0], device=device, dtype=dtype)
        self.mu = torch.tensor([0.5], device=device, dtype=dtype)
        self.sigma = torch.tensor([0.2], device=device, dtype=dtype)
        self.a = torch.tensor([4.392e-8], device=device, dtype=dtype)
        self.b = torch.tensor([3.5], device=device, dtype=dtype)
        self.c = torch.tensor([4.23e-4], device=device, dtype=dtype)
        self.d = torch.tensor([0.0], device=device, dtype=dtype)
        self.e = torch.tensor([0.0], device=device, dtype=dtype)
        self.activeFault = torch.tensor([0.0], device=device, dtype=dtype)
        self.kgScale = torch.tensor([0.101972], device=device, dtype=dtype)
        self.thrust_unit = thrust_unit
        self.intermittent_fault = True

    def set_intermittent(self, inter, mu, sigma):
        self.intermittent_fault = inter
        self.mu = mu
        self.sigma = sigma

    def set_speed(self, speed):
        self.speed = torch.tensor([speed], device=device, dtype=dtype)
        # if self.fault_mag > 0:
        #     if self.intermittent_fault:
        #         torch.normal(mean=self.mu,std=self.sigma, out=self.activeFault)
        #         if self.activeFault > 0.5:
        #             self.speed = self.speed * (1 - self.fault_mag)
        #     else:
        #         self.speed = self.speed * (1 - self.fault_mag)
        self.thrust = self.a * self.speed * \
            torch.pow(self.dia, self.b) / (torch.sqrt(self.pitch))
        self.thrust = self.thrust * (self.c * self.speed * self.pitch)

    def set_speed1(self, speed):
        self.speed = speed
        # if self.fault_mag > 0:
        #     if self.intermittent_fault:
        #         torch.normal(mean=self.mu,std=self.sigma, out=self.activeFault)
        #         if self.activeFault > 0.5:
        #             self.speed = self.speed * (1 - self.fault_mag)
        #     else:
        #         self.speed = self.speed * (1 - self.fault_mag)

        self.d = self.dia.pow(self.b).div(self.pitch.sqrt())
        self.thrust = self.a.mul((self.speed.mul(self.d))).mul(
            (self.c.mul(self.speed.mul(self.pitch))))
        # print(speed)

    def get_speed(self):
        return self.speed

    def set_fault(self, fault):
        self.fault_mag = fault


def test(iterations=100000):
    Dia = torch.tensor([10.0], device=device, dtype=dtype)
    Pitch = torch.tensor([0.3], device=device, dtype=dtype)
    P = Propeller(Dia, Pitch, device=device)
    for i in range(iterations):
        P.set_speed(i)
        # print(P.thrust)

    return


def test1(iterations=100000):
    Dia = torch.tensor([10.0], device=device, dtype=dtype)
    Pitch = torch.tensor([0.3], device=device, dtype=dtype)
    P = Propeller(Dia, Pitch, device=device)
    for i in range(iterations):
        speed = torch.tensor([i], device=device, dtype=dtype)
        P.set_speed1(speed)
        # print(P.thrust)

    return


dtype = torch.float
iter = 10000
device = "cpu"
# torch.cuda.set_device(device)
print(device)
result = timeit.timeit(stmt='test(iter)', globals=globals(), number=1)
print(result)
result = timeit.timeit(stmt='test1(iter)', globals=globals(), number=1)
print(result)

device = "cuda:0"
torch.cuda.set_device(device)
print(device)
result = timeit.timeit(stmt='test(100)', globals=globals(), number=1)
print(result)
result = timeit.timeit(stmt='test(iter)', globals=globals(), number=1)
print(result)
result = timeit.timeit(stmt='test1(iter)', globals=globals(), number=1)
print(result)

exit()


# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================
# ===================================================================================
#
# QUADCOPTER_PARAMETERS =  {
#             "position": [0, 0, 0],
#             "orientation": [0, 0, 0],
#             "L": 0.3,
#             "r": 0.1,
#             "prop_size": [10, 4.5],
#             "weight": 1.2
# }
# DefaultEnv = { "Env" : {
#     "RotorFault":{
#           "enabled": False,
#           "min_magnitude" : 0,
#           "max_magnitude" : 0.30,
#           "randomTime" : True,
#           "starttime": 500,
#           "endtime" : 31000,
#           "randomRotor" : True,
#           "faultRotorID" : 1
#     },
#     "Wind":{
#           "enabled": False,
#           "min_magnitude" : 0,
#           "max_magnitude" : 3,
#           "randomDirection" : False,
#           "direction" : 1
#       },
#       "PositionNoise": {
#           "enabled": False,
#           "min_magnitude": 0,
#           "max_magnitude" : 0.3
#       },
#       "AttitudeNoise": {
#           "enabled": False,
#           "min_magnitude" : 0,
#           "max_magnitude" : 0.1
#       }
#      }
#     }
#
# DefaultPath = {
#     "randomPath" : False,
#     "randomSeed" : 1234,
#     "randomLimit": 8,
#     "waypoints":{
#         "x": [0, 0],
#         "y": [0, 0],
#         "z": [0, 5]
#     },
#     "safetyRadius": 1,
#     "maxStepsPerRun": 3000,
#     "stablilizationAtGoal": 100
#
# }
#
# # Low altitude Model
# # transfer function for along-wind
# def u_transfer_function(height, airspeed):
#     # turbulence level defines value of wind speed in knots at 20 feet
#     # turbulence_level = 15 * 0.514444 # convert speed from knots to meters per second
#     turbulence_level = 15
#     length_u = height / ((0.177 + 0.00823 * height) ** (0.2))
#     # length_u = 1750
#     sigma_w = 0.1 * turbulence_level
#     sigma_u = sigma_w / ((0.177 + 0.000823 * height) ** (0.4))
#     num_u = torch.Tensor([sigma_u * (torch.sqrt((2 * length_u) / (torch.pi * airspeed))) * airspeed]).to(device)
#     den_u = torch.Tensor([length_u, airspeed]).to(device)
#     H_u = signal.TransferFunction(num_u, den_u)
#     return H_u
#
#
# # transfer function for cross-wind
# def v_transfer_function(height, airspeed):
#     # turbulence level defines value of wind speed in knots at 20 feet
#     # turbulence_level = 15 * 0.514444 # convert speed from knots to meters per second
#     turbulence_level = 15
#     length_v = height / ((0.177 + 0.00823 * height) ** (0.2))
#     # length_v = 1750
#     sigma_w = 0.1 * turbulence_level
#     sigma_v = sigma_w / ((0.177 + 0.000823 * height) ** (0.4))
#     b = sigma_v * (torch.sqrt((length_v) / (torch.pi * airspeed)))
#     Lv_V = length_v / airspeed
#     num_v = torch.Tensor([(torch.sqrt(3) * Lv_V * b), b]).to(device)
#     den_v = torch.Tensor([(Lv_V ** 2), 2 * Lv_V, 1]).to(device)
#     H_v = signal.TransferFunction(num_v, den_v)
#     return H_v
#
#
# # transfer function for vertical-wind
# def w_transfer_function(height, airspeed):
#     # turbulence level defines value of wind speed in knots at 20 feet
#     # turbulence_level = 15 * 0.514444 # convert speed from knots to meters per second
#     turbulence_level = 15
#     length_w = height
#     # length_w = 1750
#     sigma_w = 0.1 * turbulence_level
#     c = sigma_w * (torch.sqrt((length_w) / (torch.pi * airspeed)))
#     Lw_V = length_w / airspeed
#     num_w = torch.Tensor([(torch.sqrt(3) * Lw_V * c), c]).to(device)
#     den_w = torch.Tensor([(Lw_V ** 2), 2 * Lw_V, 1]).to(device)
#     H_v = signal.TransferFunction(num_w, den_w)
#     return H_v
#
# class GUI():
#
#     def __init__(self, quads, path , safebound, radius=1):
#         self.quads = quads
#         self.pos = []
#         self.fig = plt.figure()
#         self.ax = Axes3D.Axes3D(self.fig)
#         self.set_plot_bounds()
#         self.init_plot()
#         self.currentPoints = torch.Tensor([]).to(device)
#         self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)
#         self.scatter = 0
#         self.safe_bound = torch.Tensor(safebound).to(device)
#         self.path = path
#         self.radius = radius
#         self.min_distances_points = []
#         self.min_distances = []
#         self.update_freq = 5
#         self.update_count = 0
#         plt.ion()
#         plt.show()
#
#
#
#
#     def rotation_matrix(self,anglesArr):
#         angles = torch.Tensor(anglesArr).to(device)
#         ct = torch.cos(angles[0])
#         cp = torch.cos(angles[1])
#         cg = torch.cos(angles[2])
#         st = torch.sin(angles[0])
#         sp = torch.sin(angles[1])
#         sg = torch.sin(angles[2])
#         R_x = torch.Tensor([[1, 0, 0], [0, ct, -st], [0, st, ct]]).to(device)
#         R_y = torch.Tensor([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]).to(device)
#         R_z = torch.Tensor([[cg, -sg, 0], [sg, cg, 0], [0, 0, 1]]).to(device)
#         R = torch.mm(R_z, torch.mm(R_y, R_x))
#         return R.to(device)
#
#     def init_plot(self):
#
#         self.quads['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
#         self.quads['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
#         # self.quads['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)
#
#     def set_plot_bounds(self):
#         self.bounds = 5
#         self.ax.set_xlim3d([-self.bounds, self.bounds])
#         self.ax.set_xlabel('X')
#         self.ax.set_ylim3d([-self.bounds, self.bounds])
#         self.ax.set_ylabel('Y')
#         self.ax.set_zlim3d([0, self.bounds])
#         self.ax.set_zlabel('Z')
#         self.ax.set_title('3D Quadcopter Simulation')
#
#
#     def update(self, quad, bound):
#         self.update_count+=1
#
#         if self.update_count > self.update_freq:
#             self.update_count = 0
#             self.safe_bound = bound
#             self.quads = quad
#             self.ax.cla()
#             self.init_plot()
#             self.set_plot_bounds()
#             R = self.rotation_matrix(self.quads['orientation'])
#             L = self.quads['L']
#             points = torch.Tensor([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T.to(device)
#             points = torch.mm(R,points)
#             points[0,:] += self.quads['state'][0]
#             points[1,:] += self.quads['state'][1]
#             points[2,:] += self.quads['state'][2]
#             self.quads['l1'].set_data(points[0,0:2],points[1,0:2])
#             self.quads['l1'].set_3d_properties(points[2,0:2])
#             self.quads['l2'].set_data(points[0,2:4],points[1,2:4])
#             self.quads['l2'].set_3d_properties(points[2,2:4])
#             # self.quads['hub'].set_data(points[0,5],points[1,5])
#             # self.quads['hub'].set_3d_properties(points[2,5])
#             pos = [self.quads['state'][0],
#                    self.quads['state'][1],
#                    self.quads['state'][2]]
#             self.pos.append(pos)
#
#
#
#
#             self.updateQuadLine()
#             self.showWaypoints()
#             self.updatePathToGoal()
#             self.showDistanceToRef()
#
#             plt.pause(0.0001)
#
#     def showDistanceToRef(self):
#
#         # get closest point on the linspace between waypoints
#
#         p1 = [self.pos[-1][0], self.pos[-1][1], self.pos[-1][2]]
#         distances = []
#         points = []
#         for i in range(len(self.safe_bound)):
#             p2 = np.array([self.safe_bound[i][0], self.safe_bound[i][1], self.safe_bound[i][2]])
#             squared_dist = np.sum((p1 - p2) ** 2, axis=0)
#             dist = np.sqrt(squared_dist)
#             distances.append(dist)
#             points.append(p2)
#             # print(str(self.safe_bound[i]) +" "+ str(dist))
#
#         i = np.where(distances == np.amin(distances))
#         index = i[0][0]
#
#         self.min_distances_points.append((p1, points[index]))
#         self.min_distances.append(distances[index])
#         ind = 0
#         for (quadPoint, minDistancePoint) in self.min_distances_points:
#             xs = [quadPoint[0], minDistancePoint[0]]
#             ys = [quadPoint[1], minDistancePoint[1]]
#             zs = [quadPoint[2], minDistancePoint[2]]
#             if self.min_distances[ind] <= self.radius:
#                 color = "g"
#             else:
#                 color = "r"
#             ind += 1
#             self.ax.plot3D(xs, ys, zs, linewidth=1, alpha=0.5, c=color)
#
#         return
#
#
#     def updateQuadLine(self):
#         # x_c1 = torch.Tensor([])
#         # y_c1 = torch.Tensor([])
#         # z_c1 = torch.Tensor([])
#         x_c1 = []#torch.Tensor([])
#         y_c1 = []#torch.Tensor([])
#         z_c1 = []#torch.Tensor([])
#         pos = self.pos
#         #
#         for i in range(len(pos)):
#             # torch.cat((x_c1 , self.pos[i][0]))
#             # torch.cat((y_c1 , self.pos[i][1]))
#             # torch.cat((z_c1 , self.pos[i][2]))
#             x_c1.append(pos[i][0])
#             y_c1.append(pos[i][1])
#             z_c1.append(pos[i][2])
#
#         self.ax.plot3D(x_c1,y_c1,z_c1,linewidth=2, c="b")
#
#
#     def showWaypoints(self):
#         xs = self.path["x"]
#         ys = self.path["y"]
#         zs = self.path["z"]
#         for i in range(len(xs)):
#             self.ax.scatter(xs[i], ys[i], zs[i], marker='o', color='k', alpha=0.5, linewidth=5)
#
#     def updatePathToGoal(self):
#
#         xs = self.path['x']
#         ys = self.path['y']
#         zs = self.path['z']
#
#         self.ax.plot3D(xs, ys, zs, linewidth=1, c="k")
#
#         # using scatter plot does not accurately show the size of the safebound- its a crude estimate.
#         # path = self.safe_bound
#         # ind = 0
#         # width = 20 * self.radius
#         # for i in range(len(path)):
#         #     pos = path[i]
#         #     ind += 1
#         #     self.ax.scatter(pos[0], pos[1], pos[2], marker='o',color='green', alpha=0.05, linewidth=width)
#
#
#     def keypress_routine(self,event):
#         sys.stdout.flush()
#         if event.key == 'x':
#             y = list(self.ax.get_ylim3d())
#             y[0] += 0.2
#             y[1] += 0.2
#             self.ax.set_ylim3d(y)
#         elif event.key == 'w':
#             y = list(self.ax.get_ylim3d())
#             y[0] -= 0.2
#             y[1] -= 0.2
#             self.ax.set_ylim3d(y)
#         elif event.key == 'd':
#             x = list(self.ax.get_xlim3d())
#             x[0] += 0.2
#             x[1] += 0.2
#             self.ax.set_xlim3d(x)
#         elif event.key == 'a':
#             x = list(self.ax.get_xlim3d())
#             x[0] -= 0.2
#             x[1] -= 0.2
#             self.ax.set_xlim3d(x)
#
#     def close(self):
#
#         plt.close(self.fig)
#
# class Propeller():
#     def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
#         self.dia = torch.Tensor([prop_dia]).to(device)
#         self.pitch = torch.Tensor([prop_pitch]).to(device)
#         self.thrust_unit = thrust_unit
#         self.speed = 0 #RPM
#         self.thrust = torch.Tensor([0]).to(device)
#         self.fault_mag = 0
#         self.intermittent_fault = False
#         self.mu = 0.5 # torch.Tensor(0.5)
#         self.sigma = 0.2 #torch.Tensor(0.2)  # mean and standard deviation
#
#     def set_intermittent(self, inter , mu, sigma):
#         self.intermittent_fault = inter
#         self.mu = mu
#         self.sigma = sigma
#
#     def set_speed(self,speed):
#         self.speed = torch.Tensor([speed]).to(device)
#         if self.fault_mag > 0:
#             # print(self.fault_mag)
#             # print("Speed before :" + str(self.speed))
#             if self.intermittent_fault:
#                 active = torch.normal(mean=torch.Tensor([self.mu]).to(device), std=torch.Tensor([self.sigma]).to(device))
#                 if active > 0.5:
#                     self.speed = self.speed * torch.Tensor((1 - self.fault_mag)).to(device)
#                 else:
#                     self.speed = self.speed
#             else:
#                 self.speed = self.speed * (1 - self.fault_mag)
#             # print("Speed after :" + str(self.speed))
#
#         # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
#         self.thrust = 4.392e-8 * self.speed * torch.pow(self.dia,3.5)/(torch.sqrt(self.pitch))
#         self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
#
#         if self.thrust_unit == 'Kg':
#             self.thrust = self.thrust*0.101972
#     def get_speed(self):
#         return self.speed
#
#     def set_fault(self,fault):
#         self.fault_mag = fault
#
#
# class Quadcopter():
#     # State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
#     # From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
#     def __init__(self,
#                  Quad = QUADCOPTER_PARAMETERS ,
#                  gravity=9.81,
#                  b=0.0245,
#                  Env = DefaultEnv,
#                  Path = DefaultPath,
#                  render=False
#                  ):
#         self.lastFaultStart = 100
#         self.CTRL_TIMESTEP = Quad.control_freq
#         self.quads = Quad
#         self.g = gravity
#         self.b = b
#         self.Path = Path
#         if self.Path['randomPath']:
#             self.setRandomPath()
#         else:
#             self.setPath()
#
#         self.outsideSafezone = False
#         self.total_time_outside_safety = 0
#         self.done = False
#
#         # print(self.fault_time)
#         self.ode =  scipy.integrate.ode(self.state_dot).set_integrator('vode',nsteps=self.CTRL_TIMESTEP,method='bdf')
#         self.time = datetime.datetime.now()
#
#         self.stepNum = 0
#         self.quads['state'] = torch.zeros(12).to(device)
#         self.quads['state'][0:3] = torch.Tensor(self.quads['position']).to(device)
#         self.quads['state'][6:9] = torch.Tensor(self.quads['orientation']).to(device)
#         self.quads['m1'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
#         self.quads['m2'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
#         self.quads['m3'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
#         self.quads['m4'] = Propeller(self.quads['prop_size'][0],self.quads['prop_size'][1])
#         # From Quadrotor Dynamics and Control by Randal Beard
#         ixx=((2*self.quads['weight']*self.quads['r']**2)/5)+(2*self.quads['weight']*self.quads['L']**2)
#         iyy=ixx
#         izz=((2*self.quads['weight']*self.quads['r']**2)/5)+(4*self.quads['weight']*self.quads['L']**2)
#         self.quads['I'] = torch.Tensor([[ixx,0,0],[0,iyy,0],[0,0,izz]]).to(device)
#         self.quads['invI'] = torch.linalg.inv(self.quads['I']).to(device)
#
#         self.Env = Env
#         self.faultModes = []
#         self.setEnv()
#
#         self.rend = render
#         if self.rend: self.setupGUI()
#
#
#     def setupGUI(self):
#
#         self.GUI = GUI(self.quads, self.Path['waypoints'] , self.safe_bound, self.Path['safetyRadius'])
#
#         return
#
# #==========Path related functions =========================
#
#     def setRandomPath(self):
#         self.stepsToGoal = 0
#         # self.goals = torch.Tensor([0,0,0])
#         self.goals = []
#         self.safe_region = []
#         self.trackingAccuracy = self.Path['safetyRadius']
#         self.maxSteps = self.Path['maxStepsPerRun']
#         self.requiredStableAtGoal = self.Path['stablilizationAtGoal']
#         limit = self.Path['randomLimit']
#         number_waypoints = self.Path['number_waypoints']
#         # np.random.seed(self.Path['randomSeed'])
#         torch.manual_seed(self.Path['randomSeed'])
#         x_path = [0,0]
#         # x_path = torch.Tensor([0])
#         y_path = [0,0]
#         # y_path = torch.Tensor([0])
#         z_path = [0,5]
#         # z_path = torch.Tensor([5])
#         for i in range(number_waypoints):
#             # x_dest = torch.randint(-limit, limit, (1,))
#             # y_dest = torch.randint(-limit, limit, (1,))
#             # z_dest = torch.Tensor([5]) # np.random.randint(5,5)
#             # x_path = torch.cat((x_path , x_dest))
#             # y_path = torch.cat((y_path , y_dest))
#             # z_path = torch.cat((z_path , z_dest))
#             x_dest = np.random.randint(-limit, limit)
#             y_dest = np.random.randint(-limit, limit)
#             z_dest = 5 #np.random.randint(5,5)
#             x_path.append(x_dest)
#             y_path.append(y_dest)
#             z_path.append(z_dest)
#
#         steps = len(x_path)
#         interval_steps = 50
#
#         self.stepsToGoal=0
#         for i in range(steps):
#             if (i < steps - 1):
#                 # create linespace between waypoint i and i+1
#                 # x_lin = torch.linspace(x_path[i], x_path[i + 1], interval_steps)
#                 # y_lin = torch.linspace(y_path[i], y_path[i + 1], interval_steps)
#                 # z_lin = torch.linspace(z_path[i], z_path[i + 1], interval_steps)
#                 x_lin = np.linspace(x_path[i], x_path[i + 1], interval_steps)
#                 y_lin = np.linspace(y_path[i], y_path[i + 1], interval_steps)
#                 z_lin = np.linspace(z_path[i], z_path[i + 1], interval_steps)
#             else:
#                 # x_lin = torch.linspace(x_path[i], x_path[i], interval_steps)
#                 # y_lin = torch.linspace(y_path[i], y_path[i], interval_steps)
#                 # z_lin = torch.linspace(z_path[i], z_path[i], interval_steps)
#                 x_lin = np.linspace(x_path[i], x_path[i], interval_steps)
#                 y_lin = np.linspace(y_path[i], y_path[i], interval_steps)
#                 z_lin = np.linspace(z_path[i], z_path[i], interval_steps)
#
#             self.goals.append([x_path[i], y_path[i], z_path[i]])
#
#             # for each pos in linespace append a goal
#             # print(self.goals)
#
#             newRegion = []
#
#             for j in range(interval_steps):
#
#                 point = [x_lin[j], y_lin[j], z_lin[j]]
#                 # point = torch.Tensor([x_lin[j], y_lin[j], z_lin[j]])
#                 newRegion.append(point)
#                 self.stepsToGoal += 1
#
#             self.safe_region.append(newRegion)
#
#         self.goals = torch.Tensor(self.goals).to(device)
#         self.safe_region = torch.Tensor(self.safe_region).to(device)
#
#         self.currentWaypoint = 1
#         self.numberOfWaypoints =steps
#         self.updateWaypoint(self.currentWaypoint)
#         self.trackingAccuracy = self.Path['safetyRadius']
#         self.maxSteps = self.Path['maxStepsPerRun']
#         self.requiredStableAtGoal = self.Path['stablilizationAtGoal']
#
#         self.Path['waypoints'] = {
#             "x": x_path,
#             "y": y_path,
#             "z": z_path
#         }
#         # print(self.Path)
#         return
#
#     def setPath(self):
#
#         self.stepsToGoal = 0
#         self.goals = torch.Tensor([])
#         self.safe_region = torch.Tensor([])
#         self.trackingAccuracy = self.Path['safetyRadius']
#         self.maxSteps = self.Path['maxStepsPerRun']
#         self.requiredStableAtGoal = self.Path['stablilizationAtGoal']
#         x_path = torch.Tensor(self.Path['waypoints']['x'])
#         y_path = torch.Tensor(self.Path['waypoints']['y'])
#         z_path = torch.Tensor(self.Path['waypoints']['z'])
#         steps = len(x_path)
#         interval_steps = 50
#
#         for i in range(steps):
#             if (i < steps - 1):
#                 # create linespace between waypoint i and i+1
#                 x_lin = torch.linspace(x_path[i], x_path[i + 1], interval_steps)
#                 y_lin = torch.linspace(y_path[i], y_path[i + 1], interval_steps)
#                 z_lin = torch.linspace(z_path[i], z_path[i + 1], interval_steps)
#             else:
#                 x_lin = torch.linspace(x_path[i], x_path[i], interval_steps)
#                 y_lin = torch.linspace(y_path[i], y_path[i], interval_steps)
#                 z_lin = torch.linspace(z_path[i], z_path[i], interval_steps)
#             t = torch.Tensor([x_path[i], y_path[i], z_path[i]])
#             self.goals = torch.cat((self.goals ,t) )
#             # for each pos in linespace append a goal
#             self.safe_region = torch.cat((self.safe_region ,  torch.Tensor([])))
#             for j in range(interval_steps):
#                 self.safe_region[i] = torch.cat((self.safe_region[i] , torch.Tensor([x_lin[j], y_lin[j], z_lin[j]])))
#                 self.stepsToGoal += 1
#
#         self.numberOfWaypoints = steps
#         self.currentWaypoint = 0
#         self.updateWaypoint(self.currentWaypoint)
#         return
#
#     def isAtPosition(self,pos):
#
#         dest_x= self.goals[pos][0]
#         dest_y= self.goals[pos][1]
#         dest_z = self.goals[pos][2]
#         # exit()
#         [x, y, z ] = self.get_position()
#         x_error = dest_x - x
#         y_error = dest_y - y
#         z_error = dest_z - z
#         # print(x_error,y_error,z_error)
#
#         total_distance_to_goal = abs(x_error) + abs(y_error) + abs(z_error)
#         isAt = True if total_distance_to_goal < self.trackingAccuracy else False
#         return isAt
#
#     def updateWaypoint(self, goalIndex):
#
#         self.target = self.goals[goalIndex]
#         self.safe_bound = self.safe_region[max(goalIndex-1,0)]
#         # print(goalIndex)
#         # print(self.target)
#         # print(self.safe_bound)
#         return
#
#     def getMinDistanceToReference(self):
#         #get closest point on the line space between waypoints
#         p1 = torch.Tensor(self.get_position())
#         distances = torch.Tensor([])
#         points = torch.Tensor([])
#         for i in range(len(self.safe_bound)):
#             p2 = torch.Tensor([self.safe_bound[i][0], self.safe_bound[i][1], self.safe_bound[i][2]])
#             squared_dist = torch.sum((p1 - p2) ** 2)
#             dist = torch.sqrt(squared_dist)
#             distances = torch.cat((distances , torch.Tensor([dist])))
#             points =torch.cat((points , p2))
#
#             #print(str(self.safe_bound[i]) +" "+ str(dist))
#         i = torch.where(distances == torch.amin(distances))
#         index = i[0][0]
#         return distances[index]
#
#     def checkSafetyBound(self):
#         self.current_distance_to_opt = self.getMinDistanceToReference()
#         if  self.current_distance_to_opt > self.trackingAccuracy :
#             self.total_time_outside_safety += 1
#             self.outsideSafezone = True
#         else:
#             self.outsideSafezone = False
#
#         return
#
#
# #==========Environment related functions =========================
#
#     def randomizeQuadcopterEnvironment(self, quad):
#         # print(quad)
#         config = quad
#         count = []
#
#         if quad.RotorFault.enabled:
#             count.append(1)
#         else:
#             count.append(0)
#         if quad.Wind.enabled:
#             count.append(1)
#         else:
#             count.append(0)
#         if quad.PositionNoise.enabled:
#             count.append(1)
#         else:
#             count.append(0)
#         if quad.AttitudeNoise.enabled:
#             count.append(1)
#         else:
#             count.append(0)
#
#         if sum(count) == 0:
#             #dont trrigger faults
#             self.fault_time = self.maxSteps*2
#
#         while sum(count) > 1:
#             #TODO set sampling distribution of faults
#             selection = random.randint(0,len(count)-1)
#             if count[selection] == 1:
#                 newcount = [0] * len(count)
#                 newcount[selection] = 1
#                 count = newcount
#
#         if count[0] == 1:
#             config.RotorFault.enabled = True
#         else:
#             config.RotorFault.enabled = False
#         if count[1] == 1:
#             config.Wind.enabled = True
#         else:
#             config.Wind.enabled = False
#         if count[2] == 1:
#             config.PositionNoise.enabled = True
#         else:
#             config.PositionNoise.enabled = False
#         if count[3] == 1:
#             config.AttitudeNoise.enabled = True
#         else:
#             config.AttitudeNoise.enabled = False
#
#
#         return config
#
#     def setEnv(self):
#         #if more than one mode is selected, choose a random one
#         Env = self.randomizeQuadcopterEnvironment(self.Env)
#
#         if Env == []:
#             faultModes = ['None']
#         # ===============Rotor Fault config====================
#         keys = Env.keys()
#
#         if "RotorFault" in keys and Env['RotorFault']['enabled']:
#             # print("Rotor")
#             self.setupRotorFaults()
#         # ===============Wind gust config=================
#         if "Wind" in keys and Env['Wind']['enabled'] :
#             # print("Wind")
#             self.setupWind()
#         #============= Position Noise config===============
#         if "PositionNoise" in keys and Env['PositionNoise']['enabled']:
#             # print("PositionNoise")
#             self.setupPositionSensorNoise()
#         # ============= Attitude Noise config===============
#         if "AttitudeNoise" in keys and Env['AttitudeNoise']['enabled']:
#             # print("AttitudeNoise")
#             self.setupAttitudeSensorNoise()
#
#         return
#
#     def setupRotorFaults(self):
#         self.faultModes.append("RotorFault")
#         fault_mag = self.Env['RotorFault']['magnitude']
#         faults = torch.Tensor([0, 0, 0, 0]).to(device)
#         if bool(self.Env['RotorFault']['randomRotor']):
#             rotor = torch.randint( 0, 4, (1,))
#         else:
#             rotor = self.Env['RotorFault']['faultRotorID']
#         faults[rotor] = fault_mag
#         self.rotorFault = faults
#         if bool(self.Env['RotorFault']['randomTime']):
#             # randomly trigger in first 500 steps
#             stime = torch.randint(10,  self.lastFaultStart, (1,))
#             # etime = random.randint(int(self.maxSteps / 2), self.maxSteps)
#             etime = self.Path['maxStepsPerRun']*2 # whole episode
#             self.fault_time = stime
#             self.rotorFaultStart = stime
#             self.rotorFaultEnd = etime
#         else:
#             self.rotorFaultStart = self.Env['RotorFault']['starttime']
#             self.fault_time = self.rotorFaultStart
#             self.rotorFaultEnd = self.Env['RotorFault']['endtime']
#
#         if self.Env.RotorFault.enabled and self.Env.RotorFault.intermittent_fault :
#             self.quads['m1'].set_intermittent(True,
#                                               self.Env.RotorFault.intermittent_fault_mean,
#                                               self.Env.RotorFault.intermittent_fault_std)
#             self.quads['m2'].set_intermittent(True,
#                                               self.Env.RotorFault.intermittent_fault_mean,
#                                               self.Env.RotorFault.intermittent_fault_std)
#             self.quads['m3'].set_intermittent(True,
#                                               self.Env.RotorFault.intermittent_fault_mean,
#                                               self.Env.RotorFault.intermittent_fault_std)
#             self.quads['m4'].set_intermittent(True,
#                                               self.Env.RotorFault.intermittent_fault_mean,
#                                               self.Env.RotorFault.intermittent_fault_std)
#
#             # print("Intermittent Fault active")
#
#
#         return
#
#     def set_motor_faults(self, faults):
#         f1 = faults[0]
#         f2 = faults[1]
#         f3 = faults[2]
#         f4 = faults[3]
#         self.quads['m1'].set_fault(f1)
#         self.quads['m2'].set_fault(f2)
#         self.quads['m3'].set_fault(f3)
#         self.quads['m4'].set_fault(f4)
#
#         return
#
#     def setupPositionSensorNoise(self):
#         self.faultModes.append("PositionNoise")
#         noise =  self.Env['PositionNoise']['magnitude']
#         self.posNoiseMag = noise
#         if bool(self.Env['PositionNoise']['randomTime']):
#             # randomly trigger in first 500 steps
#             stime = torch.randint(10,  self.lastFaultStart, (1,))
#             # etime = random.randint(int(self.maxSteps / 2), self.maxSteps)
#             etime = self.Path['maxStepsPerRun'] * 2  # whole episode
#             self.fault_time = stime
#             self.PositionNoiseStart = stime
#             self.PositionNoiseEnd = etime
#         else:
#
#             self.PositionNoiseStart = self.Env['PositionNoise']['starttime']
#             self.fault_time = self.PositionNoiseStart
#
#             self.PositionNoiseEnd = self.Env['PositionNoise']['endtime']
#
#     def setupAttitudeSensorNoise(self):
#         self.faultModes.append("AttitudeNoise")
#         noise = self.Env['AttitudeNoise']['magnitude']
#         self.attNoiseMag = noise
#         stime = torch.randint(10, int(self.maxSteps / 2), (1,))
#         self.fault_time = stime
#         if bool(self.Env['AttitudeNoise']['randomTime']):
#             # randomly trigger in first 500 steps
#             stime = torch.randint(10,  self.lastFaultStart, (1,))
#             # etime = random.randint(int(self.maxSteps / 2), self.maxSteps)
#             etime = self.Path['maxStepsPerRun'] * 2  # whole episode
#             self.fault_time = stime
#             self.AttitudeNoiseStart = stime
#             self.AttitudeNoiseEnd = etime
#         else:
#             self.AttitudeNoiseStart = self.Env['AttitudeNoise']['starttime']
#             self.fault_time = self.AttitudeNoiseStart
#
#             self.AttitudeNoiseEnd = self.Env['AttitudeNoise']['endtime']
#
#     def setupWind(self):
#         self.faultModes.append("Wind")
#         if bool(self.Env['Wind']['randomDirection']):
#             direction = torch.randint(0, 4, (1,))
#         else:
#             direction = self.Env['Wind']['direction']
#
#         WindMag = self.Env['Wind']['magnitude']
#         if (direction == 0):
#             winds = [-WindMag, 0, 0]
#         elif (direction == 1):
#             winds = [WindMag, 0, 0]
#         elif (direction == 2):
#             winds = [0, -WindMag, 0]
#         else:
#             winds = [0, WindMag, 0]
#
#
#
#         self.windMag = WindMag
#         self.airspeed = 15 # default airspeed
#         self.randWind = self.generate_wind_turbulence(5)
#         # print(self.randWind)
#         self.XWind = 0
#         self.YWind = 0
#         self.ZWind = 0
#         self.setNormalWind(winds)
#         if bool(self.Env['Wind']['randomTime']):
#             # randomly trigger in first 500 steps
#             stime = torch.randint(10, self.lastFaultStart, (1,))
#             # etime = random.randint(int(self.maxSteps / 2), self.maxSteps)
#             etime = self.Path['maxStepsPerRun'] * 2  # whole episode
#             self.fault_time = stime
#             self.windStart = stime
#             self.windEnd = etime
#         else:
#             self.windStart = self.Env['Wind']['starttime']
#             self.fault_time = self.windStart
#
#             self.windEnd = self.Env['Wind']['endtime']
#
#     def setWind(self, wind_vec):
#         self.randWind = wind_vec
#
#     def setNormalWind(self, winds):
#         self.XWind = winds[0]
#         self.YWind = winds[1]
#         self.ZWind = winds[2]
#
#     def generate_wind_turbulence(self, h):
#
#         height = float(h) * 3.28084
#         airspeed = float(self.airspeed) * 3.28084
#
#         mean = 0
#         std = 1
#         # create a sequence of 1000 equally spaced numeric values from 0 - 5
#         num_samples = self.maxSteps+10
#         t_p = torch.linspace(0, 10, num_samples)
#         samples1 = 10 * torch.normal((mean, std), size=num_samples)
#         samples2 = 10 * torch.normal((mean, std), size=num_samples)
#         samples3 = 10 * torch.normal((mean, std), size=num_samples)
#
#         tf_u = u_transfer_function(height, airspeed)
#         tf_v = v_transfer_function(height, airspeed)
#         tf_w = w_transfer_function(height, airspeed)
#
#         tout1, y1, x1 = signal.lsim(tf_u, samples1, t_p)
#         # tout1, y1, x1 = signal.lsim(tf_u, n1, t_w)
#         # covert obtained values to meters/second
#         y1_f = [i * 0.305 for i in y1]
#         tout2, y2, x2 = signal.lsim(tf_v, samples2, t_p)
#         # tout2, y2, x2 = signal.lsim(tf_v, n2, t_w)
#         y2_f = [i * 0.305 for i in y2]
#         tout3, y3, x3 = signal.lsim(tf_w, samples3, t_p)
#         # tout3, y3, x3 = signal.lsim(tf_w, n3, t_w)
#         y3_f = [i * 0.305 for i in y3]
#
#         return [y1_f, y2_f, y3_f]
#
# #===============Simulation Functions ===========
#
#     def update(self, dt=0.02):
#         self.stepNum += 1
#         # print(self.stepNum)
#         # Handle Rotor Fault Here
#         if "RotorFault" in  self.faultModes:
#             if self.stepNum > self.rotorFaultStart:
#                 self.set_motor_faults(self.rotorFault)
#             if self.stepNum > self.rotorFaultEnd:
#                 self.set_motor_faults(torch.Tensor([0, 0, 0, 0]).to(device))
#
#         state = self.quads['state'].to("cpu").numpy()
#         print(state)
#         self.ode.set_initial_value(state, 0).set_f_params()
#         ## removed key from multi drone object - might cause priblems ( .set_f_params(key))
#         newState = self.ode.integrate(self.ode.t + dt)
#         print(newState)
#         self.quads['state'] = torch.Tensor(newState).to(device)
#         self.quads['state'][6:9] = self.wrap_angle(self.quads['state'][6:9])
#         self.quads['state'][2] = max(0, self.quads['state'][2])
#
#         #check if the quad reached the next waypoint or finished
#         self.updatePathPosition()
#
#     def state_dot(self, time, state):
#         state_dot = torch.zeros(12).to(device)
#         # The velocities(t+1 x_dots equal the t x_dots)
#         state_dot[0] = self.quads['state'][3]
#         state_dot[1] = self.quads['state'][4]
#         state_dot[2] = self.quads['state'][5]
#         # The acceleration
#         height = self.quads['state'][2]
#         #
#         F_d  = torch.Tensor([ 0, 0, 0]).to(device)
#         #
#         air_density = 1.225 #kg/m^3
#         C_d = 1
#         cube_width = 0.1 # 10cm x 10cm cube as shape model of quadcopter
#         A_yz = cube_width*cube_width
#         A_xz = cube_width*cube_width
#         A_xy = cube_width*cube_width
#
#         A = [ A_yz , A_xz  , A_xy  ] # cross sectional area in each axis perpendicular to velocity axis
#
#         #if wind is active the velocity in each axis is subject to wind
#         if "Wind" in self.faultModes and self.stepNum > self.fault_time:
#             nomX = self.XWind
#             nomY = self.YWind
#             nomZ = self.ZWind
#             randX = self.randWind[0][self.stepNum]
#             randY = self.randWind[1][self.stepNum]
#             randZ = self.randWind[2][self.stepNum]
#             wind_velocity_vector = torch.Tensor([ nomX + randX  , nomY + randY  , nomZ + randZ]).to(device) # wind velocity in each axis
#         else:
#             wind_velocity_vector = torch.Tensor([0 ,0 ,0]).to(device)
#         ang = self.quads['state'][6:9]
#         wind_vel_inertial_frame = torch.matmul(self.rotation_matrix(ang) ,  wind_velocity_vector)
#         V_b = torch.Tensor([ state[0], state[1] , state[2]]).to(device)
#         V_a = wind_vel_inertial_frame  - V_b
#         DragVector = torch.Tensor([
#             A[0] * (V_a[0] * abs(V_a[0])),
#             A[1] * (V_a[1] * abs(V_a[1])),
#             A[2] * (V_a[2] * abs(V_a[2]))
#         ]).to(device)
#         F_d = torch.Tensor([i * (0.5 * air_density * C_d) for i in DragVector]).to(device)
#         #Dryden turbulence random component
#             # self.windMag
#
#         x_dotdot = torch.Tensor([0,0,-self.quads['weight']*self.g]).to(device) \
#                    + torch.matmul(self.rotation_matrix(self.quads['state'][6:9]),
#                             torch.Tensor([0,0,(self.quads['m1'].thrust +
#                                            self.quads['m2'].thrust +
#                                            self.quads['m3'].thrust +
#                                            self.quads['m4'].thrust)]).to(device)) \
#                    /self.quads['weight'] + F_d
#
#         state_dot[3] = x_dotdot[0]
#         state_dot[4] = x_dotdot[1]
#         state_dot[5] = x_dotdot[2]
#         # The angular rates(t+1 theta_dots equal the t theta_dots)
#         state_dot[6] = self.quads['state'][9]
#         state_dot[7] = self.quads['state'][10]
#         state_dot[8] = self.quads['state'][11]
#         # The angular accelerations
#         omega = self.quads['state'][9:12]
#         tau = torch.Tensor([self.quads['L']*
#                         (self.quads['m1'].thrust-self.quads['m3'].thrust),
#                          self.quads['L']*(self.quads['m2'].thrust-self.quads['m4'].thrust),
#                          self.b*(self.quads['m1'].thrust-self.quads['m2'].thrust +
#                                  self.quads['m3'].thrust-self.quads['m4'].thrust)]).to(device)
#
#
#         subsubres = torch.matmul(self.quads['I'],omega).to(device)
#         subres = torch.cross(omega,subsubres).to(device)
#         print(subres)
#         print(tau)
#         res = torch.subtract(tau,  subres)
#         omega_dot = torch.matmul(self.quads['invI'].to(device), res.to(device))
#         state_dot[9] = omega_dot[0]
#         state_dot[10] = omega_dot[1]
#         state_dot[11] = omega_dot[2]
#         return state_dot
#
#     def updatePathPosition(self):
#
#         if self.stepNum >= self.maxSteps:
#             self.done = True
#             return
#
#         if (self.isAtPosition(self.currentWaypoint)):
#             # if there is another waypoint then set the quadcopters next goal and safety margin
#             if (self.currentWaypoint < self.numberOfWaypoints-1):
#                 self.currentWaypoint += 1
#                 self.updateWaypoint(self.currentWaypoint)
#             else:
#                 # there is no more waypoints in the trajectory
#                 self.stableAtGoal += 1
#                 if (self.stableAtGoal > self.requiredStableAtGoal):
#                     # wait for quadcopter to stabilize around goal
#                     self.done = True
#                 else:
#                     self.done = False
#         else:
#             #the quadcopter has not reached the goal yet
#             self.stableAtGoal = 0
#
#         self.checkSafetyBound()
#
#
# # ==== internal sim and helper functions ========
#
#     def rotation_matrix(self, anglesArr):
#         angles = anglesArr
#         ct = torch.cos(angles[0])
#         cp = torch.cos(angles[1])
#         cg = torch.cos(angles[2])
#         st = torch.sin(angles[0])
#         sp = torch.sin(angles[1])
#         sg = torch.sin(angles[2])
#         R_x = torch.Tensor([[1, 0, 0], [0, ct, -st], [0, st, ct]]).to(device)
#         R_y = torch.Tensor([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]).to(device)
#         R_z = torch.Tensor([[cg, -sg, 0], [sg, cg, 0], [0, 0, 1]]).to(device)
#         R = torch.mm(R_z, torch.mm(R_y, R_x))
#         return R
#
#     def wrap_angle(self, val):
#         return ((val + torch.pi) % (2 * torch.pi) - torch.pi)
#
#     def set_motor_speeds(self,speeds):
#
#         self.quads['m1'].set_speed(speeds[0])
#         self.quads['m2'].set_speed(speeds[1])
#         self.quads['m3'].set_speed(speeds[2])
#         self.quads['m4'].set_speed(speeds[3])
#
#     def get_motor_speeds(self):
#
#         return [self.quads['m1'].get_speed(), self.quads['m2'].get_speed(),
#                 self.quads['m3'].get_speed(), self.quads['m4'].get_speed()]
#
#     def get_motor_speeds_rpm(self):
#         return [self.quads['m1'].get_speed(), self.quads['m2'].get_speed(),self.quads['m3'].get_speed(),self.quads['m4'].get_speed()]
#
#     def get_position(self):
#         return self.quads['state'][0:3]
#
#     def get_linear_rate(self):
#         return self.quads['state'][3:6]
#
#     def get_orientation(self):
#         return self.quads['state'][6:9]
#
#     def get_angular_rate(self):
#         return self.quads['state'][9:12]
#
#     def get_state(self):
#         return self.quads['state']
#
#     def set_position(self,position):
#         self.quads['state'][0:3] = position
#
#     def set_orientation(self,orientation):
#         self.quads['state'][6:9] = orientation
#
#     def get_time(self):
#         return self.time
#
#     def get_target(self):
#         #needed for controllers - next reference point
#         return self.target
#
# # ========Safe Control Repo Interface Functions ==========
#
#     def get_fault_time(self):
#
#         return  self.fault_time
#
#
#     def reset(self):
#
#         if self.Path['randomPath']:
#             self.setRandomPath()
#         else:
#             self.setPath()
#
#         self.setEnv()
#
#
#         self.total_time_outside_safety = 0
#         self.done = False
#         self.time = datetime.datetime.now()
#         self.stepNum = 0
#         self.quads['state'] = torch.zeros(12)
#         self.quads['state'][0:3] = self.quads['position']
#         self.quads['state'][6:9] = self.quads['orientation']
#         self.quads['m1'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
#         self.quads['m2'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
#         self.quads['m3'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
#         self.quads['m4'] = Propeller(self.quads['prop_size'][0], self.quads['prop_size'][1])
#         # From Quadrotor Dynamics and Control by Randal Beard
#         ixx = ((2 * self.quads['weight'] * self.quads['r'] ** 2) / 5) + (
#                     2 * self.quads['weight'] * self.quads['L'] ** 2)
#         iyy = ixx
#         izz = ((2 * self.quads['weight'] * self.quads['r'] ** 2) / 5) + (
#                     4 * self.quads['weight'] * self.quads['L'] ** 2)
#         self.quads['I'] = torch.Tensor([[ixx, 0, 0], [0, iyy, 0], [0, 0, izz]])
#         self.quads['invI'] = torch.linalg.inv(self.quads['I'])
#         if self.rend:
#             self.GUI.close()
#             self.setupGUI()
#
#         obs = self._get_obs()
#         info = self._get_info()
#         rew = self._get_rew()
#         done = self._get_done()
#
#         if self.rend: self.render()
#
#         return obs, info
#
#     def step(self, action):
#         self.set_motor_speeds(action)
#         self.update()
#         # if self.stepNum == self.fault_time:
#             # print("Fault Occured")
#
#         obs = self._get_obs()
#         info = self._get_info()
#         rew = self._get_rew()
#         done = self._get_done()
#
#         if self.rend : self.render()
#
#         return obs, rew, done, info
#
#     def render(self):
#
#         if self.rend: self.GUI.update(self.quads, self.safe_bound)
#         if self.done: self.GUI.close()
#
#         return
#
#     def _get_obs(self):
#         obs = torch.clone(torch.Tensor(self.get_state()))
#         #apply the noise as a sudden fault
#         if self.stepNum > self.fault_time:
#             if "PositionNoise" in self.faultModes:
#                 x_noise = torch.rand(-self.posNoiseMag, self.posNoiseMag)
#                 y_noise = torch.rand(-self.posNoiseMag, self.posNoiseMag)
#                 z_noise = torch.rand(-self.posNoiseMag, self.posNoiseMag)
#                 obs[0] += x_noise
#                 obs[1] += y_noise
#                 obs[2] += z_noise
#             if "AttitudeNoise" in self.faultModes:
#                 roll_noise = torch.rand(-self.attNoiseMag, self.attNoiseMag)
#                 pitch_noise = torch.rand(-self.attNoiseMag, self.attNoiseMag)
#                 yaw_noise = torch.rand(-self.attNoiseMag, self.attNoiseMag)
#                 obs[6] += roll_noise
#                 obs[7] += pitch_noise
#                 obs[8] += yaw_noise
#
#         obs = torch.cat((obs, self.target))
#         # print(obs)
#         return obs
#     def _get_info(self):
#         #calculate error - distance to safezone
#         distance = self.getMinDistanceToReference() - self.trackingAccuracy
#         #negative distance means inside safezone
#         if distance < 0:
#             distance_to_safezone = 0
#         else:
#             distance_to_safezone = distance
#         info = {"mse": self.getMinDistanceToReference(),
#                 "distance_to_safezone": distance_to_safezone ,
#                 "outside_safezone": 1 if self.outsideSafezone else 0
#                 }
#         return info
#
#     def _get_rew(self):
#         rew = -1 if self.outsideSafezone else 0
#         return rew
#
#     def _get_done(self):
#         # check if quad is at final waypoint
#         if (self.stepNum > self.maxSteps):
#             self.done = True
#         return self.done
#
#
#     def close(self):
#         if self.rend:
#             self.GUI.close()
#         return
