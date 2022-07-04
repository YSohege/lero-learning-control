"""Model Predictive Control.

"""
import numpy as np
import casadi as cs
import pybullet as p

from copy import deepcopy

from safe_control_gym.controllers.base_controller import BaseController
from safe_control_gym.envs.benchmark_env import Task
from safe_control_gym.envs.constraints import ConstraintList, GENERAL_CONSTRAINTS, create_constraint_list
from scipy.stats import dirichlet as DirichletDistribution




class RBCMPC(BaseController):
    """MPC with full nonlinear model.

    """
    DefaultHorizon = np.array([5,5])
    DefaultQ_MPC =  np.array([[1],[1]])
    DefaultR_MPC = np.array([[1],[1]])

    def __init__(
            self,
            env_func,
            horizon=DefaultHorizon,
            q_mpc=DefaultQ_MPC,
            r_mpc=DefaultR_MPC,
            warmstart=True,
            output_dir="results/temp",
            additional_constraints=None,
            **kwargs
            ):
        """Creates task and controller.

        Args:
            env_func (Callable): function to instantiate task/environment.
            horizon (int): mpc planning horizon.
            q_mpc (list): diagonals of state cost weight.
            r_mpc (list): diagonals of input/action cost weight.
            warmstart (bool): if to initialize from previous iteration.
            output_dir (str): output directory to write logs and results.
            additional_constraints (list): List of additional constraints

        """
        self.numberControllers = len(horizon)

        self.models = []
        self.q_mpcs = q_mpc
        self.r_mpcs = r_mpc

        self.warmstart =False
        for k, v in locals().items():
            if k != "self" and k != "kwargs" and "__" not in k:
                self.__dict__.update({k: v})
        # Task.
        self.env = env_func()
        self.reference = self.env._get_reset_info()['x_reference']
        # Setup reference input.
        if self.env.TASK == Task.STABILIZATION:
            self.mode = "stabilization"
            self.x_goal = self.env.X_GOAL
        elif self.env.TASK == Task.TRAJ_TRACKING:
            self.mode = "tracking"
            self.traj = self.env.X_GOAL.T
            # Step along the reference.
            self.traj_step = 0

        if self.env.constraints is not None:
            if additional_constraints is not None:
                additional_ConstraintsList = create_constraint_list(additional_constraints,
                                                                    GENERAL_CONSTRAINTS,
                                                                    self.env)
                self.additional_constraints = additional_ConstraintsList.constraints
                self.reset_constraints(self.env.constraints.constraints + self.additional_constraints)
            else:
                self.reset_constraints(self.env.constraints.constraints)
                self.additional_constraints = []
        else:
            self.constraints = ConstraintList([])
            self.state_constraints_sym = self.constraints.get_state_constraint_symbolic_models()
            self.input_constraints_sym = self.constraints.get_input_constraint_symbolic_models()

        for controller in range(self.numberControllers):
            hor = horizon[controller]
            q  = self.q_mpcs[controller]
            r  = self.r_mpcs[controller]

            # Model parameters
            model = self.env.symbolic
            dt = model.dt
            T = hor
            Q = self.get_cost_weight_matrix(q, model.nx)
            R = self.get_cost_weight_matrix(r, model.nu)
            modelConfig = { "model" : model,
                            "dt" : dt,
                            "T" : T,
                            "Q" : Q,
                            "R" : R,
                            "x_prev": None,
                            "u_prev": None
                            }
            self.models.append( modelConfig )



        self.set_dynamics_func()
        # CasADi optimizer.
        self.setup_optimizer()


        # initilize distribution with uniform density based on attitude controller set size
        self.alpha = [2] * self.numberControllers
        self.quantiles = [1 / self.numberControllers] * self.numberControllers
        self.AttitudeControllerDistribution = DirichletDistribution
        self.AttitudeControllerDistribution.pdf(self.quantiles, self.alpha)

        # self.reset()

    def get_cost_weight_matrix(self,weights,
                               dim
                               ):
        """Gets weight matrix from input args.

        """
        if len(weights) == dim:
            W = np.diag(weights)
        elif len(weights) == 1:
            W = np.diag(weights * dim)
        else:
            raise Exception("Wrong dimension for cost weights.")
        return W


    def reset_constraints(self,
                          constraints
                          ):
        """ Setup the constraints list.

        Args:
            constraints (list): List of constraints controller is subject to.

        """
        self.constraints = ConstraintList(constraints)
        self.state_constraints_sym = self.constraints.get_state_constraint_symbolic_models()
        self.input_constraints_sym = self.constraints.get_input_constraint_symbolic_models()
        if len(self.constraints.input_state_constraints) > 0:
            raise NotImplementedError('MPC cannot handle combined state input constraints yet.')

    def add_constraints(self,
                        constraints
                        ):
        """Add the constraints (from a list) to the system.

        Args:
            constraints (list): List of constraints controller is subject too.

        """
        self.reset_constraints(constraints + self.constraints.constraints)

    def remove_constraints(self,
                           constraints
                           ):
        """Remove constraints from the current constraint list.

        Args:
            constraints (list): list of constraints to be removed.

        """
        old_constraints_list = self.constraints.constraints
        for constraint in constraints:
            assert constraint in self.constraints.constraints,\
                ValueError("This constraint is not in the current list of constraints")
            old_constraints_list.remove(constraint)
        self.reset_constraints(old_constraints_list)

    def close(self):
        """Cleans up resources.

        """
        self.env.close()

    def reset(self):
        """Prepares for training or evaluation.

        # """
        # self.env.close()
        # self.env = self.env_func()
        initial_obs, initial_info = self.env.reset()

        self.reference = initial_info['x_reference']



        # Setup reference input.
        if self.env.TASK == Task.STABILIZATION:
            self.mode = "stabilization"
            self.x_goal = self.env.X_GOAL
        elif self.env.TASK == Task.TRAJ_TRACKING:
            self.mode = "tracking"
            self.traj = self.env.X_GOAL.T
            # Step along the reference.
            self.traj_step = 0
            for i in range(0,  self.reference.shape[0], 10):
                p.addUserDebugLine(lineFromXYZ=[self.reference[i - 10, 0], 0, self.reference[i - 10, 2]],
                                   lineToXYZ=[self.reference[i, 0], 0, self.reference[i, 2]],
                                   lineColorRGB=[1, 0, 0],
                                   physicsClientId=self.env.PYB_CLIENT)

        self.newModels = []
        for controller in range(self.numberControllers):
            hor = self.models[controller]["T"]
            q = self.q_mpcs[controller]
            r = self.r_mpcs[controller]

            # Model parameters
            model = self.env.symbolic
            dt = model.dt
            T = hor
            Q = self.get_cost_weight_matrix(q, model.nx)
            R = self.get_cost_weight_matrix(r, model.nu)
            modelConfig = {"model": model,
                           "dt": dt,
                           "T": T,
                           "Q": Q,
                           "R": R,
                           "x_prev": None,
                           "u_prev": None
                           }
            self.newModels.append(modelConfig)

        self.models = self.newModels
        self.set_dynamics_func()
        # CasADi optimizer.
        self.setup_optimizer()

        self.reset_results_dict()
        return initial_obs, initial_info


    def set_dynamics_func(self):
        """Updates symbolic dynamics with actual control frequency.

        """
        for controller in range(self.numberControllers):

            model = self.models[controller]['model']
            dt =self.models[controller]['dt']
            dynamics_func = cs.integrator('fd', model.integration_algo,
                                               {
                                                'x': model.x_sym,
                                                'p': model.u_sym,
                                                'ode': model.x_dot
                                                },
                                               {'tf': dt})

            self.models[controller]["dynamics_func"] = dynamics_func


    def setup_optimizer(self):
        """Sets up nonlinear optimization problem.

        """
        for controller in range(self.numberControllers):

            model = self.models[controller]['model']
            T =self.models[controller]['T']
            Q =self.models[controller]['Q']
            R =self.models[controller]['R']
            dynamics_func = self.models[controller]["dynamics_func"]
            nx, nu = model.nx, model.nu
            T = T
            # Define optimizer and variables.
            opti = cs.Opti()
            # States.
            x_var = opti.variable(nx, T + 1)
            # Inputs.
            u_var = opti.variable(nu, T)
            # Initial state.
            x_init = opti.parameter(nx, 1)
            # Reference (equilibrium point or trajectory, last step for terminal cost).
            x_ref = opti.parameter(nx, T + 1)
            # Cost (cumulative).
            cost = 0
            cost_func = model.loss
            for i in range(T):
                # Can ignore the first state cost since fist x_var == x_init.
                cost += cost_func(x=x_var[:, i],
                                  u=u_var[:, i],
                                  Xr=x_ref[:, i],
                                  Ur=np.zeros((nu, 1)),
                                  Q=Q,
                                  R=R)["l"]
            # Terminal cost.
            cost += cost_func(x=x_var[:, -1],
                              u=np.zeros((nu, 1)),
                              Xr=x_ref[:, -1],
                              Ur=np.zeros((nu, 1)),
                              Q=Q,
                              R=R)["l"]
            opti.minimize(cost)
            # Constraints
            for i in range(T):
                # Dynamics constraints.
                next_state = dynamics_func(x0=x_var[:, i], p=u_var[:, i])['xf']
                opti.subject_to(x_var[:, i + 1] == next_state)
                for state_constraint in self.state_constraints_sym:
                    opti.subject_to(state_constraint(x_var[:,i]) < 0)
                for input_constraint in self.input_constraints_sym:
                    opti.subject_to(input_constraint(u_var[:,i]) < 0)
            # Final state constraints.
            for state_constraint in self.state_constraints_sym:
                opti.subject_to(state_constraint(x_var[:, i]) < 0)
            # Initial condition constraints.
            opti.subject_to(x_var[:, 0] == x_init)
            # Create solver (IPOPT solver in this version)
            opts = {"ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0}
            opti.solver('ipopt', opts)
            opti_dict = {
                "opti": opti,
                "x_var": x_var,
                "u_var": u_var,
                "x_init": x_init,
                "x_ref": x_ref,
                "cost": cost
            }

            self.models[controller]['opti_dict'] = opti_dict

        return


    def select_action(self,
                      obs
                      ):
        """Solves nonlinear mpc problem to get next action.

        Args:
            obs (np.array): current state/observation. 
        
        Returns:
            np.array: input/action to the task/env. 

        """
        actions = []
        for controller in range(self.numberControllers):
            opti_dict =self.models[controller]['opti_dict']
            x_prev =self.models[controller]['x_prev']
            u_prev = self.models[controller]['u_prev']
            opti = opti_dict["opti"]
            x_var = opti_dict["x_var"]
            u_var = opti_dict["u_var"]
            x_init = opti_dict["x_init"]
            x_ref = opti_dict["x_ref"]
            cost = opti_dict["cost"]
            # Assign the initial state.
            opti.set_value(x_init, obs)
            # Assign reference trajectory within horizon.
            goal_states = self.get_references(controller)
            opti.set_value(x_ref, goal_states)
            if self.mode == "tracking":
                self.traj_step += 1
            # Initial guess for optimization problem.
            if self.warmstart and x_prev is not None and u_prev is not None:
                # shift previous solutions by 1 step
                x_guess = deepcopy(x_prev)
                u_guess = deepcopy(u_prev)
                x_guess[:, :-1] = x_guess[:, 1:]
                u_guess[:-1] = u_guess[1:]
                opti.set_initial(x_var, x_guess)
                opti.set_initial(u_var, u_guess)
            # Solve the optimization problem.
            sol = opti.solve()
            x_val, u_val = sol.value(x_var), sol.value(u_var)
            self.models[controller]['x_prev'] = x_val
            self.models[controller]['u_prev'] = u_val


            # Take the first action from the solved action sequence.
            if u_val.ndim > 1:
                action = u_val[:, 0]
            else:
                action = np.array([u_val[0]])

            actions.append(action)

        action = self._supervisoryControl( actions)

        # self.results_dict['horizon_states'].append(deepcopy(x_val))
        # self.results_dict['horizon_inputs'].append(deepcopy(x_val))

        return action

    def get_references(self, controller):
        """Constructs reference states along mpc horizon.(nx, T+1).

        """
        T = self.models[controller]["T"]
        if self.env.TASK == Task.STABILIZATION:
            # Repeat goal state for horizon steps.
            goal_states = np.tile(self.env.X_GOAL.reshape(-1, 1), (1, T + 1))
        elif self.env.TASK == Task.TRAJ_TRACKING:
            # Slice trajectory for horizon steps, if not long enough, repeat last state.
            start = min(self.traj_step, self.traj.shape[-1])
            end = min(self.traj_step + T + 1, self.traj.shape[-1])
            remain = max(0, T + 1 - (end - start))
            goal_states = np.concatenate([
                self.traj[:, start:end],
                np.tile(self.traj[:, -1:], (1, remain))
            ], -1)
        else:
            raise Exception("Reference for this mode is not implemented.")
        return goal_states  # (nx, T+1).

    def reset_results_dict(self):
        """

        """
        self.results_dict = { 'obs': [],
                              'reward': [],
                              'done': [],
                              'info': [],
                              'action': [],
                              'horizon_inputs': [],
                              'horizon_states': []
        }

    def _supervisoryControl(self, rpms):
        self.Blend = self.AttitudeControllerDistribution.rvs(self.alpha, size=1)[0]
        # print(self.Blend)
        weightedActions = []
        index = 0
        for rpm in rpms:
            weight = [self.Blend[index]] * len(rpm)
            weightedActions.append(np.multiply(weight, rpm))
            index += 1
        action = list(map(sum, zip(*weightedActions)))
        action = np.array(action)
        return action


    def run(self,
            env=None,
            render=False,
            logging=False,
            max_steps=100
            ):
        """Runs evaluation with current policy.
        
        Args:
            render (bool): if to do real-time rendering. 
            logging (bool): if to log on terminal.
            
        Returns:
            dict: evaluation statisitcs, rendered frames. 

        """
        if env is None:
            env = self.env


        obs, info =self.reset()

        # print("Init State:")
        # print(obs)
        ep_returns, ep_lengths = [], []
        frames = []
        self.reset_results_dict()
        self.results_dict['obs'].append(obs)
        i = 0
        if self.env.TASK == Task.STABILIZATION:
            MAX_STEPS = max_steps
        elif self.env.TASK == Task.TRAJ_TRACKING:
            MAX_STEPS = self.traj.shape[1]
        else:
            raise("Undefined Task")
        self.terminate_loop = False
        while np.linalg.norm(obs - env.X_GOAL) > 1e-3 and i < MAX_STEPS and not(self.terminate_loop):
            action = self.select_action(obs)
            # print(action)

            if self.terminate_loop:
                print("Infeasible MPC Problem")
                break
            # Repeat input for more efficient control.
            obs, reward, done, info = env.step(action)
            self.results_dict['obs'].append(obs)
            self.results_dict['reward'].append(reward)
            self.results_dict['done'].append(done)
            self.results_dict['info'].append(info)
            self.results_dict['action'].append(action)
            # print(i, '-th step.')
            # print(action)
            # print(obs)
            # print(reward)
            # print(done)
            # print(info)
            # print()
            if render:
                env.render()
                frames.append(env.render("rgb_array"))
            i += 1
        # Collect evaluation results.
        ep_lengths = np.asarray(ep_lengths)
        ep_returns = np.asarray(ep_returns)
        if logging:
            msg = "****** Evaluation ******\n"
            msg += "eval_ep_length {:.2f} +/- {:.2f} | eval_ep_return {:.3f} +/- {:.3f}\n".format(
                ep_lengths.mean(), ep_lengths.std(), ep_returns.mean(),
                ep_returns.std())
        self.results_dict['obs'] = np.vstack(self.results_dict['obs'])
        try:
            self.results_dict['reward'] = np.vstack(self.results_dict['reward'])
            self.results_dict['action'] = np.vstack(self.results_dict['action'])
        except ValueError:
            raise Exception("[ERROR] mpc.run().py: MPC could not find a solution for the first step given the initial conditions. "
                  "Check to make sure initial conditions are feasible.")
        return deepcopy(self.results_dict)
