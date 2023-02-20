# Lero Learning Control 

The "Lero Learning Control" repository is a dedicated hub for sharing experimental source code and results related to learning control. This repository is part of a larger effort funded by Science Foundation Ireland, through "Grant No," to support software research and development at [Lero](https://lero.ie/) - University College Cork, a world-renowned software research center.

The repository provides a centralized and replicable location for researchers, practitioners, and other interested parties to access and utilize the experimental source code and results contained within. This allows for improved collaboration and greater transparency in the field of learning control, facilitating further research and experimentation.

If you find the "Lero Learning Control" repository helpful in your own publications, experiments, or other research, the creators kindly request that you consider citing their work in your own publications. This citation will help to promote and support the continued development of this valuable resource for the learning control community:
```
@misc{leroLearningControl2021,
  title={{Lero Learning Control Repository}},
  author={Yves Sohege, Gregory Provan},
  howpublished = {\url{https://github.com/YSohege/lero-learning-control}},
  year={2021}
}
```


## List of Implemented Environments and Controllers
### 2 Dimensional Quadcopter Environment - based on [Safe Control Gym](https://github.com/utiasDSL/safe-control-gym)
- [Constraint Barrier Functions - CBF](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/cbf)
- [Linear Quadratic Regulator - LQR](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/lqr)
- [Model Predictive Controller - MPC](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/mpc)
- [Proportional Integral Derivative - PID](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/pid)
- [Proximal Policy Optimization - PPO](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/ppo)
- [Robust Adversarial Reinforcement Learning - RARL ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/rarl)
- [Soft Actor Critic - SAC](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/sac)
- [SafeExplorer](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/safeexplorer)
- [Randomized Blended MPC Control - RBCMPC](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/rbcmpc)
- [Randomized Blended PID COntrol - RBCPID](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/rbcpid)


### 3 Dimensional Quadcopter Environment 
- [Switched PID Control](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter3D/optimal_mmacpid)
- [Multiple Model Adaptive PID Control - MMACPID](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter3D/mmacpid)
- [Randomized Blended PID Control ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter3D/rbcpid)
- [Fixed Distribution Randomized Blended PID Control](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter3D/fixed_dist_rbcpid)
- [Uniform Randomized Blended Control](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter3D/uniform_rbcpid)
- [Reinforcement Learning Randomized Blended PID Control - RLRBCPID ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter3D/rl_rbcpid)

## List of Tasks
Tasks require a Controller / Environment pair and return an performance metric after completion. 
1) [Blended Control Task (3D Quadcopter / RBCPID Controller)](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tasks/Blended_Control_Task)- Set blending weigth at each step - used for reinforcement learning
2) [Fixed Distribution Blended Control Task (3D Quadcopter / Fixed Distribution RBCPID Controller) ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tasks/Quadcopter3D_Fixed_Distribution_RBC)- Evaluate the tracking performance of RBCPID using a fixed distribution
3) [Optimal Switching Control Task (3D Quadcopter / Switched PID Controller)](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tasks/Quadcopter3D_Optimal_Switching) - Evaluate the tracking performance of Switching Control with no fault identification delay (optimal baseline) 
4) [RL Distribuition for Blended Control (3D Quadcopter / RLRBCPID Controller) ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tasks/Quadcopter3D_RL_Distribution_RBC)- Learn optimal randomized blended control distribution 
5) [Simple MMAC Quadcopter tracking (3D Quadcopter / MMACPID Controller)  ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tasks/Quadcopter3D_Trajectory_Tracking)- Simple tractory tracking task
6) [Uniform RBCPID Quadcopter tracking (3D Quadcopter / Uni RBCPID Controller)  ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tasks/Quadcopter3D_Uniform_RBC)- Evaluate the tracking performance of uniformly sampled blending weights. 


## List of Tuning Algorithms
Tuning algorithms execute Tasks repeadedly to maximize the achieved performance through controller parameter optimization. 

1) [Clustered Particle Filtering (3D Quadcopter / PID Controller) - CPF ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tuning/ClusteredParticleFiltering)
2) [Reinforcement Learning Blended Control (3D Quadcopter / RLRBCPID Controller) - RL Blending](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tuning/LearningBlendedControl)
3) [Particle Swarm Optimization(3D Quadcopter / PID Controller) - PSO ](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/tuning/ParticleSwarmOptimization)



## List of Example Experiments
Experiments run the Tuning algorithms or Tasks in a systematic way based on their configurations. The experiments are repeatable and all random seeds are fixed in the config files.
1) [2D Quadcopter - RBCMPC versus RBCPID trajectory tracking performance](https://github.com/YSohege/lero-learning-control/tree/main/experiments/Experiment1_2DQuadcopter_Randomized_PID_versus_Randomized_MPC)
2) [3D Quadcopter - Switched Control](https://github.com/YSohege/lero-learning-control/tree/main/experiments/Experiment2_3DQuadcopter_MMAC_PID)
3) [3D Quadcopter - Data Generation under various fault and disturbances](https://github.com/YSohege/lero-learning-control/tree/main/experiments/Experiment3_3DQuadcopter_DataGeneration)
4) [3D Quadcopter - MPC Test](https://github.com/YSohege/lero-learning-control/tree/main/experiments/Experiment4_QuadSim_MPC_Test)
5) [3D Quadcopter - Controller Tuning using Clustered Particle Filtering (CPF)](https://github.com/YSohege/lero-learning-control/tree/main/experiments/Experiment5-CPF-3DQuadcopter)
6) [Execution speed test - numpy versus torch / CPU versus GPU ](https://github.com/YSohege/lero-learning-control/tree/main/experiments/Experiment6_Speedtest)
7) [3D Quadcopter - Simple MPC control for trajectory tracking](https://github.com/YSohege/lero-learning-control/tree/main/experiments/Experiment7_MPC)






## Setup and Dependencies


[//]: # (NOTE ADD EXTRA MPC PATH INSTALL) 

### Windows
1) Install Python 3.x from the official Python website: https://www.python.org/downloads/windows/
2) Open Command Prompt or PowerShell as an administrator.
3) Navigate to the directory where you want to install Lero Learning Control. 
4) Clone the repository:
```
git clone https://github.com/YSohege/lero-learning-control.git
```
5) Navigate to the repository:
```
cd lero_learning_control
```
6) Install the required libraries:
```
python setup.py install
```


### Linux
1) Install Python 3.x using your system's package manager. For example, on Ubuntu, you can use the following command:
```
sudo apt-get install python3
```
2) Open a terminal window.
3) Navigate to the directory where you want to install Lero Learning Control.
4) Clone the repository:
```
git clone https://github.com/YSohege/lero-learning-control.git
```
5) Navigate to the repository:
```
cd lero_learning_control
```
6) Install the required libraries:
```
python setup.py install
```
Note that you may need to use python3 / pip3 instead of python / pip on some Linux distributions.



