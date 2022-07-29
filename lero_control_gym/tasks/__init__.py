from lero_control_gym.utils.registration import register


register(id="Quadcopter3D_Trajectory_Tracking",
         entry_point="lero_control_gym.tasks.Quadcopter3D_Trajectory_Tracking.Quadcopter3D_Trajectory_Tracking:Task",
         config_entry_point="lero_control_gym.tasks.Quadcopter3D_Trajectory_Tracking:Quadcopter3D_Trajectory_Tracking.yaml")

register(id="Blended_Control_Task",
         entry_point="lero_control_gym.tasks.Blended_Control_Task.Blended_Control_Task:Blended_Control_Task",
         config_entry_point="lero_control_gym.tasks.Blended_Control_Task:Blended_Control_Task.yaml")