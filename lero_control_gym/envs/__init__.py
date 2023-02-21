"""Register environments.

"""
from lero_control_gym.utils.registration import register

register(id="test_speed",
         entry_point="lero_control_gym.envs.test_speed.test:Test",
         config_entry_point="lero_control_gym.envs.test_speed:test.yaml")

register(id="cartpole",
         entry_point="lero_control_gym.envs.gym_control.cartpole:CartPole",
         config_entry_point="lero_control_gym.envs.gym_control:cartpole.yaml")

register(
    id="quadcopter2D",
    entry_point="lero_control_gym.envs.quadcopter2D.quadcopter:Quadcopter",
    config_entry_point="lero_control_gym.envs.quadcopter2D:quadcopter.yaml")

register(
    id="quadcopter3D",
    entry_point="lero_control_gym.envs.quadcopter3D.quadcopter3D:Quadcopter",
    config_entry_point="lero_control_gym.envs.quadcopter3D:quadcopter3D.yaml")


register(id="network_underwater_ROV",
         entry_point="lero_control_gym.envs.Networked_Underwater_ROV.Networked_Underwater_ROV:Networked_Underwater_ROV",
         config_entry_point="lero_control_gym.envs.Networked_Underwater_ROV:Networked_Underwater_ROV.yaml")
