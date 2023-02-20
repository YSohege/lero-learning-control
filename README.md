# Lero Learning Control 

This repository is by [Yves Sohege](https://github.com/YSohege) and produced under the "Grant No"
as a unified place to host experimental source code and results in a replicable way. 

[Lero](https://lero.ie/) is a world leading software research centre funded by Science Foundation Ireland.


If you find this repository helpful in your publications, experimentations or other research,
please consider citing :

```
@misc{leroLearningControl2021,
  title={{Lero Learning Control Repository}},
  author={Yves Sohege, Gregory Provan},
  howpublished = {\url{https://github.com/YSohege/lero-learning-control}},
  year={2021}
}
```

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




## List of Implemented Controllers and Environments
###2 Dimensional Quadcopter Environment
- [CBF](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/cbf)
- [LQR](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/lqr)
- [mpc](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/mpc)
- [mpsc](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/mpsc)
- [pid](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/pid)
- [ppo](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/ppo)
- [rarl](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/rarl)
- [rbcmpc](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/rbcmpc)
- [rbcpid](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/rbcpid)
- [sac](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/sac)
- [safeexplorer](https://github.com/YSohege/lero-learning-control/tree/main/lero_control_gym/controllers/Quadcopter2D/safeexplorer)
