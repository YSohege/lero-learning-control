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
### Windows
Install Python 3.x from the official Python website: https://www.python.org/downloads/windows/

Open Command Prompt or PowerShell as an administrator.

Navigate to the directory where you want to install Lero Learning Control.

Clone the repository:
```
git clone https://github.com/YSohege/lero-learning-control.git
```
Navigate to the repository:
```
cd lero_learning_control
```
Install the required libraries:
```
python setup.py install
```


### Linux
Install Python 3.x using your system's package manager. For example, on Ubuntu, you can use the following command:
```
sudo apt-get install python3
```

Open a terminal window.

Navigate to the directory where you want to install Lero Learning Control.

Clone the repository:
```
git clone https://github.com/YSohege/lero-learning-control.git
```

Navigate to the repository:
```
cd lero_learning_control
```

Install the required libraries:
```
python setup.py install
```

Note that you may need to use pyhton3/pip3 instead of python/pip on some Linux distributions.





[//]: # ()
[//]: # (+ Python/numpy/[PyTorch]&#40;https://pytorch.org&#41;)

[//]: # (+ [locuslab/mpc.pytorch]&#40;https://github.com/locuslab/mpc.pytorch&#41;)

[//]: # ()
[//]: # (# LQR Imitation Learning Experiments)

[//]: # ()
[//]: # (From within the `imitation_lqr` directory:)

[//]: # (1. `train.py` is the main training script for the experiment )

[//]: # (   in Section 5.3.)

[//]: # ()
[//]: # (# Non-Convex Imitation Learning Experiments)

[//]: # ()
[//]: # (From within the `imitation_nonconvex` directory:)

[//]: # (1. `make_dataset.py` should be run to create a dataset of trajectories)

[//]: # (   for each environment.)

[//]: # (2. `il_exp.py` is the main training script for each experiment.)

[//]: # (3. `run-pendulum-cartpole.sh` runs all of the experiments for the)

[//]: # (   pendulum and cartpole environments in Section 5.3.)

[//]: # (3. `run-complex-pendulum.sh` runs all of the experiments for the)

[//]: # (   non-realizable pendulum environment in Section 5.4.)
