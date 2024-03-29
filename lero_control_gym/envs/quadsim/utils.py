import numpy as np
import os

from quad import DeterministicQuad
from quad import StochasticQuad
from quad import linear_quad_dynamics
from quad import nonlinear_quad_dynamics

from plotter import Plotter


class History:
    def __init__(self, env_name):
        self.env_name = env_name

# Note that before running, these LD Library Path needs to be configured


def check_mpc_hsl_solver_in_path():
    # os.path.dirname(os.path.abspath(__file__)) +
    hsl_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) +
                               "/../src/controllers/hsl/lib")

    if not any(
        hsl_path in x
            for x in os.environ['LD_LIBRARY_PATH'].split(':')):

        print("MPC HSL Solver in LD_LIBRARY_PATH does not found.")
        print(
            "Please run this command in terminal at " +
            "working directory root folder:"
        )
        print(hsl_path)
        print(
            r'export LD_LIBRARY_PATH=' +
            r'"$LD_LIBRARY_PATH:' + hsl_path + r'"')
        exit(1)


def calculateControllerMetrics(env):
    resp_final = env.history.sol_x[0][-1]
    tol = 5e-2
    tl_ind = 0
    th_ind = 0
    ts_ind = 0
    tp = 0
    vl = 0.10 * resp_final
    vh = 0.90 * resp_final
    vs_l = 0.98 * resp_final
    vs_h = 1.02 * resp_final
    peak_val = 0
    for ind, val in enumerate(env.history.sol_x[0]):
        if abs(val - vl) < tol:
            tl_ind = ind
            break
    for ind, val in enumerate(env.history.sol_x[0]):
        if abs(val - vh) < tol:
            th_ind = ind
            break
    for ind, val in enumerate(env.history.sol_x[0]):
        if ts_ind != 0:
            if vs_l > val or vs_h < val:
                ts_ind = 0
        if vs_l < val and vs_h > val and ts_ind == 0:
            ts_ind = ind
        if val > peak_val:
            peak_val = val
            tp = ind
    rise_time = env.history.sol_t[0][th_ind] - env.history.sol_t[0][tl_ind]
    settling_time = env.history.sol_t[0][ts_ind]
    overshoot = (peak_val - resp_final) * 100 / resp_final
    peak = peak_val
    peak_time = env.history.sol_t[0][tp]
    ss_error = abs(env.reference_state[0] - resp_final)
    total_rew = env.history.sol_reward.sum()

    print("Env-Shape: ", env.t.shape, env.history.sol_ref.shape,
          env.history.sol_x.shape, env.history.sol_reward.shape)
    print("Rise time: %1.3f sec" % rise_time)
    print("Settling time: %1.3f sec" % settling_time)
    print("Overshoot: %2.3f percent" % overshoot)
    print("Peak time: %1.3f sec" % peak_time)
    print("Steady State Error: %2.3f rad" % ss_error)
    print("Total Reward: %2.3f" % total_rew)
    print("---------------------------------------------------------------")
    print("---------------------------------------------------------------")
    return rise_time, settling_time, overshoot, peak, peak_time


def simulate_envs(controller, env1, env2, env3, env4, num_episode):
    for epi in range(num_episode):
        obs = env1.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if isinstance(action, tuple):
                action = action[0]
            obs, reward, done, _ = env1.step(action)

        obs = env2.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if isinstance(action, tuple):
                action = action[0]
            obs, reward, done, _ = env2.step(action)

        obs = env3.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if isinstance(action, tuple):
                action = action[0]
            obs, reward, done, _ = env3.step(action)

        obs = env4.reset()
        done = False
        while not done:
            action = controller.predict(obs, deterministic=True)
            if isinstance(action, tuple):
                action = action[0]
            obs, reward, done, _ = env4.step(action)


def createEnvs(t_end, simulation_freq,
               control_freq, random_state_seed,
               set_custom_u_limit,
               custom_u_high,
               set_constant_reference,
               constant_reference,
               dynamics_state=np.array([3.14 / 4, 0, 0, 0, 0, 0]),
               eval_env=True):

    # Linear deterministic quadcopter
    env1 = DeterministicQuad(linear_quad_dynamics, t_end=t_end,
                             simulation_freq=simulation_freq,
                             control_freq=control_freq, random_state_seed=0,
                             dynamics_state=dynamics_state,
                             set_custom_u_limit=set_custom_u_limit,
                             custom_u_high=custom_u_high,
                             set_constant_reference=True,
                             constant_reference=constant_reference,
                             eval_env=eval_env)
    # Linear stochastic quadcopter
    env2 = StochasticQuad(linear_quad_dynamics, t_end=t_end,
                          simulation_freq=simulation_freq,
                          control_freq=control_freq, random_state_seed=0,
                          dynamics_state=dynamics_state,
                          set_custom_u_limit=set_custom_u_limit,
                          custom_u_high=custom_u_high,
                          set_constant_reference=True,
                          constant_reference=constant_reference,
                          eval_env=eval_env)
    # Nonlinear deterministic quadcopter
    env3 = DeterministicQuad(nonlinear_quad_dynamics, t_end=t_end,
                             simulation_freq=simulation_freq,
                             control_freq=control_freq, random_state_seed=0,
                             dynamics_state=dynamics_state,
                             set_custom_u_limit=set_custom_u_limit,
                             custom_u_high=custom_u_high,
                             set_constant_reference=True,
                             constant_reference=constant_reference,
                             eval_env=eval_env)
    # Nonlinear stochastic quadcopter
    env4 = StochasticQuad(nonlinear_quad_dynamics, t_end=t_end,
                          simulation_freq=simulation_freq,
                          control_freq=control_freq, random_state_seed=0,
                          dynamics_state=dynamics_state,
                          set_custom_u_limit=set_custom_u_limit,
                          custom_u_high=custom_u_high,
                          set_constant_reference=True,
                          constant_reference=constant_reference,
                          eval_env=eval_env)
    return env1, env2, env3, env4


def test_controller(controller, t_end, plot=False, save_plot=False,
                    constant_reference=None):
    control_freq = 50
    simulation_freq = 250
    t_end = 5

    if constant_reference is None:
        constant_reference = np.array([1, 0, 1, 0, 1, 0])
    custom_u_high = np.array([0.1, 0.1, 0.1])

    env1, env2, env3, env4 =\
        createEnvs(t_end=t_end,
                   simulation_freq=simulation_freq,
                   control_freq=control_freq,
                   random_state_seed=0,
                   set_custom_u_limit=False,
                   custom_u_high=custom_u_high,
                   set_constant_reference=True,
                   constant_reference=constant_reference)

    num_episode = 1
    simulate_envs(controller, env1, env2, env3, env4, num_episode)
    calculateControllerMetrics(env1)

    if plot:
        plotter = Plotter(type(controller).__name__)
        plotter.plot_only_specific_element(env1, env2, env3, env4,
                                           save_plot=save_plot)
        plotter.plot_only_specific_element(env1, env2, env3, env4,
                                           save_plot=save_plot, axis=1)
        plotter.plot_all_with_reference(env1, env2, env3, env4,
                                        save_plot=save_plot)
        plotter.plot_reward(env1, env2, env3, env4, save_plot=save_plot)
        plotter.plot_actions(env1, env2, env3, env4, save_plot=save_plot)
        if plot:
            plotter.show()
    return env1, env2, env3, env4
