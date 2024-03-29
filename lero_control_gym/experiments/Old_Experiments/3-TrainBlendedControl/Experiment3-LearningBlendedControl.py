import torch as th
from stable_baselines3 import PPO
from lero_control_gym.tasks.Blended_Control_Task.Blended_Control_Task import Blended_Control_Task
from lero_control_gym.utils.configuration import ConfigFactory
import random
import os
# Create an environment


def main():
    # Create an environment
    CONFIG_FACTORY = ConfigFactory()

    config = CONFIG_FACTORY.merge()

    while True:
        try:
            env = Blended_Control_Task(
                config.Task.quadcopter3D, config.Task.rbc_pid)

            policy_kwargs = dict(activation_fn=th.nn.LeakyReLU, net_arch=[
                                 dict(pi=[32, 16, 8], vf=[32, 16, 8])])

            random.seed(config.Task.random_seed)
            model = PPO(
                "MlpPolicy",
                env,
                policy_kwargs=policy_kwargs,
                verbose=1,
                tensorboard_log="./trainingLog/")
            # model = PPO.load("blendingAgent", env=env)
            model.learn(total_timesteps=config.Task.number_steps)

            files = [f for f in os.listdir("trainedAgentsAAAI23")]
            model.save("./trainedAgentsAAAI23/Agent-" + str(len(files)))

        except Exception:
            files = [f for f in os.listdir("trainedAgentsAAAI23")]
            model.save("./trainedAgentsAAAI23/InterruptedAgent-" +
                       str(len(files)))

    return


if __name__ == "__main__":
    main()
