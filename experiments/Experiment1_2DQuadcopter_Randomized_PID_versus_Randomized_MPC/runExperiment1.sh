#!/bin/bash


## Tracking PID
python3 ./2DQuadcopter-MMAC-PID.py --overrides ./config_pid.yaml

## Tracking MPC

python3 ./2DQuadcopter-MMAC-MPC.py --overrides ./config_mpc.yaml
