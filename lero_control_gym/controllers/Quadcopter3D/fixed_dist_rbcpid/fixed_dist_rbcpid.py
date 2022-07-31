"""Multiple Model adaptive PID control class for 3D Quadcopter
   env using simple deviation switching logic and 2 controllers.
"""

import numpy as np
import math
from scipy.spatial.transform import Rotation
from munch import munchify
from scipy.stats import dirichlet as DirichletDistribution

from lero_control_gym.controllers.base_controller import BaseController
from lero_control_gym.envs.benchmark_env import Task

DefaultPositionController = np.array([
             np.array([300, 300, 7000]),
             np.array([0.04, 0.04, 4.5]),
             np.array([450, 450, 5000])
     ])

DefaultAttitudeControllerSet = [
                        [
                            [24000, 24000, 1500],
                            [0, 0, 1.2],
                            [12000, 12000, 0]
                        ], [
                            [4000, 4000, 1500],
                            [0, 0, 1.2],
                            [1500, 1500, 0]
                        ]
                ]


class FIXED_DIST_RBCPID(BaseController):
    """ PID Class.

    """

    def __init__(self,
                 env_func=None,
                 POSITION_CONTROLLER=DefaultPositionController,
                 ATTITUDE_CONTROLLER_SET= DefaultAttitudeControllerSet,
                 RBC_DISTRIBUTION_PARAMETERS = [1]* len(DefaultAttitudeControllerSet),
                 Z_LIMITS=[0,10000],
                 TILT_LIMITS = [-10, 10],
                 MOTOR_LIMITS = [0, 9000],
                 Z_XY_OFFSET = 500,
                 YAW_CONTROL_LIMITS = [-900, 900],
                 YAW_RATE_SCALER = 0.18,
                 SEED = 87463,
                 **kwargs
                 ):
        """Common control classes __init__ method.

        Args
            g (float, optional): The gravitational acceleration in m/s^2.

        """

        super().__init__(env_func, **kwargs)

        self.env = env_func()
        self.random_seed = SEED

        self.PositionController = np.array(POSITION_CONTROLLER)
        self.AttitudeControllerSets = np.array(ATTITUDE_CONTROLLER_SET)
        self.numberControllers = len(self.AttitudeControllerSets)
        #integral position controller
        self.xi_term= 0
        self.yi_term=0
        self.zi_term=0
        self.thetai_term = 0
        self.phii_term = 0
        self.gammai_term = 0
        self.yaw_target = 0 # yaw rotation not tracked

        #max throttle
        self.Z_XY_OFFSET  = Z_XY_OFFSET
        self.MOTOR_LIMITS = MOTOR_LIMITS
        self.TILT_LIMITS = [(TILT_LIMITS[0]/180.0)*3.14,(TILT_LIMITS[1]/180.0)*3.14]
        self.YAW_CONTROL_LIMITS = YAW_CONTROL_LIMITS
        self.Z_LIMITS = [self.MOTOR_LIMITS[0] + self.Z_XY_OFFSET, self.MOTOR_LIMITS[1] - self.Z_XY_OFFSET]

        self.YAW_RATE_SCALER = YAW_RATE_SCALER
        # print(self.PositionController)
        # print(self.AttitudeControllerSets)
        self.positionErrors = [[0,0,0]]
        self.attitudeErrors = [[0,0,0]]

        # initilize distribution with uniform density based on attitude controller set size
        if RBC_DISTRIBUTION_PARAMETERS == None:
            self.alpha = [1] * self.numberControllers
        else:

            if not len(ATTITUDE_CONTROLLER_SET) == len(RBC_DISTRIBUTION_PARAMETERS):
                print("Controller set and Distribution not same size")
                exit()

            self.alpha = RBC_DISTRIBUTION_PARAMETERS
        print(self.alpha)
        self.quantiles = [1 / self.numberControllers] * self.numberControllers #must sum to 1
        self.AttitudeControllerDistribution = DirichletDistribution(self.alpha, seed=self.random_seed)
        self.AttitudeControllerDistribution.pdf(self.quantiles)




        self.obs, self.info = self.env.reset()
        self.rew = 0
        self.done = False

        self.results_dict = {'obs': [],
                             'reward': [],
                             'done': [],
                             'info': [],
                             'action': [],
                             }

        # self.reset()

    def run(self,
            iterations,
            **kwargs
            ):

        action = np.zeros(4)

        for i in range(iterations):
            # Step the environment and print all returned information.
            obs, reward, done, info = self.env.step(action)

            if done or self.done:
                break
                # self.env.reset()
                # print("Resetting Env")
                # action = np.zeros(4)
                # obs, reward, done, info = self.env.step(action)

            state = obs[0:12]
            target = obs[12:15]



            throttle, target_euler, pos_e = self._PIDPositionControl(state, target)

            rpms = self._PIDAttitudeControl(state, throttle, target_euler)

            action = self._supervisoryControl(rpms)

            self.updateResultDict(obs, reward, done, info, action)

        self.close_results_dict()

        return self.results_dict

    def step(self, action):

        self.alpha = action
        self.AttitudeControllerDistribution = DirichletDistribution(self.alpha, seed=self.random_seed)
        self.AttitudeControllerDistribution.pdf(self.quantiles)

        #apply previous action to env

        state = self.obs[0:12]
        target = self.obs[12:15]

        throttle, target_euler, pos_e = self._PIDPositionControl(state, target)

        rpms = self._PIDAttitudeControl(state, throttle, target_euler)

        #Blended RPM commands from all Attitude controllers based on action input
        self.control_action = self._supervisoryControl(rpms)

        self.obs, self.rew, self.done, self.info = self.env.step(self.control_action)

        self.updateResultDict(self.obs, self.rew, self.done, self.info , self.control_action)

        return self.obs, self.rew, self.done, self.info

    def _PIDPositionControl(self,
                               state,
                               dest_pos
                               ):
        """
        Returns:
            float: The target thrust along the drone z-axis.
            ndarray: (3,1)-shaped array of floats containing the target roll, pitch, and yaw.
            float: The current position error.

        """
        cur_pos = state[0:3]
        cur_vel = state[3:6]
        cur_orientation = state[6:9]


        x_error = dest_pos[0] - cur_pos[0]
        y_error = dest_pos[1] - cur_pos[1]
        z_error = dest_pos[2] - cur_pos[2]

        self.positionErrors.append([x_error,y_error,z_error])
        # print("Pos Errors: X= " + str(x_error) +" Y= " +str(y_error )+ " Z=" + str(z_error))
        self.xi_term += self.PositionController[1][0] * x_error
        self.yi_term += self.PositionController[1][1] * y_error
        self.zi_term += self.PositionController[1][2] * z_error

        dest_x_dot = self.PositionController[0][0] * (x_error) + self.PositionController[2][0] * (-cur_vel[0]) + self.xi_term
        dest_y_dot = self.PositionController[0][1] * (y_error) + self.PositionController[2][1] * (-cur_vel[1]) + self.yi_term
        dest_z_dot = self.PositionController[0][2] * (z_error) + self.PositionController[2][2] * (-cur_vel[2]) + self.zi_term

        throttle = np.clip(dest_z_dot, self.Z_LIMITS[0], self.Z_LIMITS[1])

        dest_theta = dest_x_dot * math.sin(cur_orientation[2]) - dest_y_dot * math.cos(cur_orientation[2])
        dest_phi = dest_x_dot * math.cos(cur_orientation[2]) + dest_y_dot * math.sin(cur_orientation[2])

        # --------------------
        # get required attitude states
        dest_gamma = self.yaw_target # fixed at 0
        dest_theta, dest_phi = np.clip(dest_theta, self.TILT_LIMITS[0], self.TILT_LIMITS[1]), \
                               np.clip(dest_phi,   self.TILT_LIMITS[0], self.TILT_LIMITS[1])

        target_euler = [dest_theta, dest_phi, dest_gamma]
        pos_e = abs(x_error) + abs(y_error) +abs(z_error)

        return throttle, target_euler, pos_e

    def _PIDAttitudeControl(self,
                            state,
                            throttle,
                            target_euler
                            ):
        """
        Returns:
            ndarray: (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.

        """

        theta, phi,  gamma = state[6:9]
        theta_dot,phi_dot, gamma_dot = state[9:12]
        dest_theta, dest_phi, dest_gamma = target_euler

        theta_error = dest_theta - theta
        phi_error = dest_phi - phi
        gamma_dot_error =(self.YAW_RATE_SCALER*self._wrap_angle(dest_gamma-gamma)) - gamma_dot

        self.attitudeErrors.append([theta_error,phi_error,gamma_dot])

        rpms = []

        for controller in self.AttitudeControllerSets:
            p = controller[0]
            i = controller[1]
            d = controller[2]

            x_val = (p[0] * (theta_error)) + (d[0] * (-theta_dot))
            y_val = (p[1] * (phi_error) + d[1] * (-phi_dot))
            z_val = (p[2] * (gamma_dot_error))
            z_val = np.clip(z_val, self.YAW_CONTROL_LIMITS[0], self.YAW_CONTROL_LIMITS[1])

            # calculate motor commands depending on controller selection
            m1 = throttle + x_val + z_val
            m2 = throttle + y_val - z_val
            m3 = throttle - x_val + z_val
            m4 = throttle - y_val - z_val

            rpm = np.clip([m1, m2, m3, m4], self.MOTOR_LIMITS[0], self.MOTOR_LIMITS[1])

            rpms.append(rpm)

        return rpms

    def _supervisoryControl(self, rpms):

        self.Blend = self.AttitudeControllerDistribution.rvs(size=1)[0]

        weightedActions = []
        index = 0
        for rpm in rpms:
            weight = [self.Blend[index]] * len(rpm)
            weightedActions.append(np.multiply(weight, rpm))
            index += 1
        action = list(map(sum, zip(*weightedActions)))
        action = np.array(action)
        return action


    def close(self):
        """Cleans up resources.

        """
        self.env.close()

    def close_results_dict(self):
        """Cleanup the rtesults dict and munchify it.

        """
        self.results_dict['obs'] = np.vstack(self.results_dict['obs'])
        self.results_dict['reward'] = np.vstack(self.results_dict['reward'])
        self.results_dict['done'] = np.vstack(self.results_dict['done'])
        self.results_dict['info'] = np.vstack(self.results_dict['info'])
        self.results_dict['action'] = np.vstack(self.results_dict['action'])

        self.results_dict = munchify(self.results_dict)

    def reset(self):
        """Resets the control classes.

        The previous step's and integral errors for both position and attitude are set to zero.

        """
        # self.env = self.env_func()
        initial_obs, initial_info = self.env.reset()
        self.xi_term = 0
        self.yi_term = 0
        self.zi_term = 0
        self.thetai_term = 0
        self.phii_term = 0
        self.gammai_term = 0


        self.results_dict = {'obs': [],
                             'reward': [],
                             'done': [],
                             'info': [],
                             'action': [],
                             }
        return initial_obs, initial_info

    def updateResultDict(self, obs, reward, done, info, action):
        self.results_dict['obs'].append(obs)
        self.results_dict['reward'].append(reward)
        self.results_dict['done'].append(done)
        self.results_dict['info'].append(info)
        self.results_dict['action'].append(action)

    def printState(self, obs, reward, done, info,action, target_euler):





        print("---------State---------")
        print("Position [X,Y,Z] = " + str(obs[0:3]))
        print("Target [X, Y, Z] =" + str(obs[12:15]))
        print("Position Error [X, Y, Z] =" + str(self.positionErrors[-1]))


        print("Attitude [Roll, Pitch, Yaw] =" + str(obs[6:9]))
        print("Attitude Target [Roll, Pitch, Yaw] =" + str(target_euler))
        print("Attitude Error [Roll, Pitch, Yaw] =" + str(self.attitudeErrors[-1]))

        print("Velocity [X,Y,Z] =" + str(obs[3:6]))
        print("Attitude Rate  [Roll, Pitch, Yaw] =" + str(obs[9:12]))

    def _wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )