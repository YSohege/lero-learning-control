import numpy as np
import math
from munch import munchify
import torch
from mpc import mpc

from lero_control_gym.controllers.base_controller import BaseController


class IMPC(BaseController):
    class Dynamics(torch.nn.Module):

        def __init__(self,
                     nx,
                     nu,
                     nhidden,
                     action_lows,
                     action_highs,
                     initial_data,
                     train_epochs,
                     bootstrap_epochs):
            self.numberStates = nx
            self.numberActions = nu
            self.actionLows = action_lows
            self.actionHighs = action_highs
            self.dataset = torch.tensor(initial_data)
            self.train_epochs = train_epochs
            self.bootstrap_epochs = bootstrap_epochs

            self.network = torch.nn.Sequential(
                torch.nn.Linear(nx + nu, nhidden),
                torch.nn.Tanh(),
                torch.nn.Linear(nhidden, nhidden),
                torch.nn.Tanh(),
                torch.nn.Linear(nhidden, nx)
            ).double()

            if not self.dataset == []:
                self.train()

        def forward(self, state, action):
            u = torch.clamp(action, self.actionLows, self.actionHighs)
            if state.dim() is 1 or u.dim() is 1:
                state = state.view(1, -1)
                u = u.view(1, -1)
            if u.shape[1] > 1:
                u = u[:, 0].view(-1, 1)
            xu = torch.cat((state, u), dim=1)
            state_residual = self.network(xu)
            next_state = state + state_residual
            return next_state

        def true_dynamics(self, state, action):

            # copy env func
            # set copy to state , step with action
            # return result
            return state

        def train(self, new_data=None):
            if new_data is not None:
                self.dataset = torch.cat((self.dataset, new_data), dim=0)
            # train on the whole dataset (assume small enough we can train on
            # all together)
            XU = self.dataset.detach()
            #
            # dtheta = angular_diff_batch(XU[1:, 0], XU[:-1, 0])
            # dtheta_dt = XU[1:, 1] - XU[:-1, 1]
            # Y = torch.cat((dtheta.view(-1, 1), dtheta_dt.view(-1, 1)), dim=1)  # x' - x residual
            # XU = XU[:-1]  # make same size as Y

            # thaw network
            for param in self.network.parameters():
                param.requires_grad = True

            optimizer = torch.optim.Adam(self.network.parameters())
            for epoch in range(self.train_epochs):
                optimizer.zero_grad()
                # MSE loss
                Yhat = self.network(XU)
                loss = (Y - Yhat).norm(2, dim=1) ** 2
                loss.mean().backward()
                optimizer.step()
                # logger.debug("ds %d epoch %d loss %f", dataset.shape[0], epoch, loss.mean().item())

            # freeze network
            for param in self.network.parameters():
                param.requires_grad = False

            return

        def get_accuracy(self):
            # Nv = 1000
            # statev = torch.cat(((torch.rand(Nv, 1, dtype=torch.double) - 0.5) * 2 * math.pi,
            # #                     (torch.rand(Nv, 1, dtype=torch.double) - 0.5) * 16), dim=1)ACTION_LOW
            # actionv = (torch.rand(Nv, self.numberActions, dtype=torch.double) - 0.5) * (self.actionHighs - self.actionLows)
            # # evaluate network against true dynamics
            # yt = self.true_dynamics(statev, actionv)
            # yp = self.dynamics(statev, actionv)
            # dtheta = angular_diff_batch(yp[:, 0], yt[:, 0])
            # dtheta_dt = yp[:, 1] - yt[:, 1]
            # E = torch.cat((dtheta.view(-1, 1), dtheta_dt.view(-1, 1)), dim=1).norm(dim=1)
            # print("Error with true dynamics theta %f theta_dt %f norm %f", dtheta.abs().mean(),
            #       dtheta_dt.abs().mean(), E.mean())

            acc = 0
            return acc

    def __init__(self,
                 env_func=None,
                 NUM_STATES=2,
                 NUM_ACTIONS=1,
                 ACTION_LOWS=[-2.0],
                 ACTION_HIGHS=[2.0],
                 TIMESTEPS=30,
                 LQR_ITER=10,
                 N_BATCH=1,
                 H_UNITS=32,
                 TRAIN_EPOCH=300,
                 RETRAIN_CYCLE=50,
                 INIT_DATA=[],
                 RENDER=True,
                 **kwargs
                 ):
        super().__init__(env_func, **kwargs)
        self.env = env_func()

        self.device = torch.device(
            "cuda:0" if torch.cuda.is_available() else "cpu")
        self.dtype = torch.double
        self.retrain_cycle = RETRAIN_CYCLE
        self.ctrl_penalty = 0.001
        # new hyperparameters for approximate dynamics
        self.model = self.Dynamics(NUM_STATES,
                                   NUM_ACTIONS,
                                   H_UNITS,
                                   ACTION_LOWS,
                                   ACTION_HIGHS,
                                   INIT_DATA,
                                   TRAIN_EPOCH
                                   )
        self.num_states = NUM_STATES
        self.num_actions = NUM_ACTIONS
        self.horizon = TIMESTEPS
        self.lqr_iter = LQR_ITER
        self.n_batches = N_BATCH
        self.actionLows = ACTION_LOWS
        self.actionHighs = ACTION_HIGHS
        self.goal = torch.zeros(self.num_states)
        self.render = RENDER
        return

    def cost_function(self):
        # goal_weights = torch.ones(self.num_states)  # equal weights
        # goal_state = torch.tensor(self.goal, dtype=self.dtype)  # nx
        #
        # q = torch.cat((
        #     goal_weights,
        #     self.ctrl_penalty * torch.ones(self.num_actions, dtype=dtype)
        # ))  # nx + nu
        # px = -torch.sqrt(goal_weights) * goal_state
        # p = torch.cat((px, torch.zeros(nu, dtype=dtype)))
        # Q = torch.diag(q).repeat(TIMESTEPS, N_BATCH, 1, 1)  # T x B x nx+nu x nx+nu
        # p = p.repeat(TIMESTEPS, N_BATCH, 1)
        # cost = mpc.QuadCost(Q, p)
        return cost

    def run(self, n_episodes=1000):
        # run MPC
        self.collected_dataset = torch.zeros(
            (self.retrain_cycle, self.num_states + self.num_actions), dtype=self.dtype)
        self.total_reward = 0
        self.action = torch.zeros(self.num_actions)
        u_init = None
        for i in range(n_episodes):
            self.state = self.env.state.copy()
            # self.state = torch.tensor(self.state, dtype=self.dtype).view(self.num_states,1)
            # command_start = time.perf_counter()
            # recreate controller using updated u_init (kind of wasteful
            # right?)
            ctrl = mpc.MPC(
                self.num_states,
                self.num_actions,
                self.horizon,
                u_lower=self.actionLows,
                u_upper=self.actionHighs,
                lqr_iter=self.lqr_iter,
                exit_unconverged=False,
                eps=1e-2,
                n_batch=self.n_batches,
                backprop=False,
                verbose=-1,
                u_init=u_init,
                grad_method=mpc.GradMethods.AUTO_DIFF)

            # compute action based on current state, dynamics, and cost
            nominal_states, nominal_actions, nominal_objs = ctrl(
                self.state, self.cost_function(), self.model)
            self.action = nominal_actions[0]  # take first planned action
            u_init = torch.cat((nominal_actions[1:], torch.zeros(
                1, self.n_batches, self.num_actions, dtype=self.dtype)), dim=0)

            # elapsed = time.perf_counter() - command_start
            s, r, _, _ = self.env.step(self.action.detach().numpy())
            self.total_reward += r
            # logger.debug("action taken: %.4f cost received: %.4f time taken: %.5fs", action, -r, elapsed)
            if self.render:
                self.env.render()

            di = i % self.retrain_cycle
            if di == 0 and i > 0:
                self.model.train(self.collected_dataset)
            self.collected_dataset[di,
                                   :self.num_states] = torch.tensor(self.state)
            self.collected_dataset[di, self.num_actions:] = self.action

    def reset(self):

        return
