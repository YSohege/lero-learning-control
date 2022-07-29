"""Register controllers.

"""
from lero_control_gym.utils.registration import register
#===============2D Quadcopter controllers ==================

register(id="quad2d_lqr",
         entry_point="lero_control_gym.controllers.Quadcopter2D.lqr.lqr:LQR",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.lqr:lqr.yaml")

register(id="quad2d_ilqr",
         entry_point="lero_control_gym.controllers.Quadcopter2D.lqr.ilqr:iLQR",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.lqr:ilqr.yaml")

register(id="quad2d_cbf",
         entry_point="lero_control_gym.controllers.Quadcopter2D.cbf.cbf_qp:CBF_QP",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.cbf:cbf_qp.yaml")

register(id="quad2d_mpc",
         entry_point="lero_control_gym.controllers.Quadcopter2D.mpc.mpc:MPC",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.mpc:mpc.yaml")

register(id="quad2d_linear_mpc",
         entry_point="lero_control_gym.controllers.Quadcopter2D.mpc.linear_mpc:LinearMPC",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.mpc:linear_mpc.yaml")

register(id="quad2d_gp_mpc",
         entry_point="lero_control_gym.controllers.Quadcopter2D.mpc.gp_mpc:GPMPC",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.mpc:gp_mpc.yaml")

register(id="quad2d_mpsc",
         entry_point="lero_control_gym.controllers.Quadcopter2D.mpsc.mpsc:MPSC",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.mpsc:mpsc.yaml")

register(id="quad2d_pid",
         entry_point="lero_control_gym.controllers.Quadcopter2D.pid.pid:PID",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.pid:pid.yaml")

register(id="quad2d_ppo",
         entry_point="lero_control_gym.controllers.Quadcopter2D.ppo.ppo:PPO",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.ppo:ppo.yaml")

register(id="quad2d_sac",
         entry_point="lero_control_gym.controllers.Quadcopter2D.sac.sac:SAC",
         config_entry_point="lero_control_gym.Quadcopter2D.controllers.sac:sac.yaml")

register(id="quad2d_safe_explorer_ppo",
         entry_point="lero_control_gym.controllers.Quadcopter2D.safe_explorer.safe_ppo:SafeExplorerPPO",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.safe_explorer:safe_ppo.yaml")

register(id="quad2d_rarl",
         entry_point="lero_control_gym.controllers.Quadcopter2D.rarl.rarl:RARL",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.rarl:rarl.yaml")

register(id="quad2d_rap",
         entry_point="lero_control_gym.controllers.Quadcopter2D.rarl.rap:RAP",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.rarl:rap.yaml")

register(id="quad2d_rbcpid",
         entry_point="lero_control_gym.controllers.Quadcopter2D.rbcpid.rbcpid:RBCPID",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.rbcpid:rbcpid.yaml")

register(id="quad2d_rbcmpc",
         entry_point="lero_control_gym.controllers.Quadcopter2D.rbcmpc.rbcmpc:RBCMPC",
         config_entry_point="lero_control_gym.controllers.Quadcopter2D.rbcmpc:rbcmpc.yaml")

#===============3D Quadcopter controllers ==================
register(id="quad3d_mmacpid",
         entry_point="lero_control_gym.controllers.Quadcopter3D.mmacpid.mmacpid:MMACPID",
         config_entry_point="lero_control_gym.controllers.Quadcopter3D.mmacpid:mmacpid.yaml")

register(id="quad3d_rbcpid",
         entry_point="lero_control_gym.controllers.Quadcopter3D.rbcpid.rbcpid:RBCPID",
         config_entry_point="lero_control_gym.controllers.Quadcopter3D.rbcpid:rbcpid.yaml")

