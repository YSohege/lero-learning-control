import torch as th
from stable_baselines3 import PPO
from lero_control_gym.tasks.Blended_Control_Task.Blended_Control_Task import Blended_Control_Task
from lero_control_gym.utils.configuration import ConfigFactory

# Create an environment


def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()

    env = Blended_Control_Task(config.Task.quadcopter3D, config.Task.rbc_pid)

    policy_kwargs = dict(activation_fn=th.nn.ReLU,
                         net_arch=[dict(pi=[32, 32], vf=[32, 32])])

    model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs,
                verbose=1, tensorboard_log="./trainingLog/")
    model.learn(total_timesteps=10000000)
    model.save("blendingAgent")

    exit()
    return


if __name__ == "__main__":
    main()

    # self.epcount = 0
    # self.action_space = spaces.MultiDiscrete(np.array([self.dirHigh, self.marginHigh]))
    # self.action_space = spaces.Discrete(self.dirHigh)
    #
    # self.low_obs = 0
    # self.high_obs = 1
    # self.input_height = 5  # one candle
    # self.input_width =
    # self.low_state = [[self.low_obs for col in range(self.input_width)] for row in range(self.input_height)]
    # self.high_state = [[self.high_obs for col in range(self.input_width)] for row in range(self.input_height)]
    # self.observation_space = spaces.Box(low=self.low_obs, high=self.high_obs, shape=(1, len(self.low_state)))


def main():

    print(result)

    return


if __name__ == "__main__":
    main()
