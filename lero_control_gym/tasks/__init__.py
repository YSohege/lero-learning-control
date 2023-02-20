from lero_control_gym.utils.registration import register


register(
    id="Quadcopter3D_Trajectory_Tracking",
    entry_point="lero_control_gym.tasks.Quadcopter3D_Trajectory_Tracking.Quadcopter3D_Trajectory_Tracking:Task",
    config_entry_point="lero_control_gym.tasks.Quadcopter3D_Trajectory_Tracking:Quadcopter3D_Trajectory_Tracking.yaml")

register(
    id="Quadcopter3D_Optimal_Switching",
    entry_point="lero_control_gym.tasks.Quadcopter3D_Optimal_Switching.Quadcopter3D_Optimal_Switching:Task",
    config_entry_point="lero_control_gym.tasks.Quadcopter3D_Optimal_Switching:Quadcopter3D_Optimal_Switching.yaml")

register(
    id="Blended_Control_Task",
    entry_point="lero_control_gym.tasks.Blended_Control_Task.Blended_Control_Task:Blended_Control_Task",
    config_entry_point="lero_control_gym.tasks.Blended_Control_Task:Blended_Control_Task.yaml")

register(
    id="Quadcopter3D_Uniform_RBC",
    entry_point="lero_control_gym.tasks.Quadcopter3D_Uniform_RBC.Quadcopter3D_Uniform_RBC:Task",
    config_entry_point="lero_control_gym.tasks.Quadcopter3D_Uniform_RBC:Quadcopter3D_Uniform_RBC.yaml")

register(
    id="Quadcopter3D_Fixed_Dist_RBC",
    entry_point="lero_control_gym.tasks.Quadcopter3D_Fixed_Distribution_RBC.Quadcopter3D_Fixed_Distribution_RBC:Task",
    config_entry_point="lero_control_gym.tasks.Quadcopter3D_Fixed_Distribution_RBC:Quadcopter3D_Fixed_Distribution_RBC.yaml")

register(
    id="Quadcopter3D_RL_Distribution_RBC",
    entry_point="lero_control_gym.tasks.Quadcopter3D_RL_Distribution_RBC.Quadcopter3D_RL_Distribution_RBC:Quadcopter3D_RL_Distribution_RBC",
    config_entry_point="lero_control_gym.tasks.Quadcopter3D_RL_Distribution_RBC:Quadcopter3D_RL_Distribution_RBC.yaml")
