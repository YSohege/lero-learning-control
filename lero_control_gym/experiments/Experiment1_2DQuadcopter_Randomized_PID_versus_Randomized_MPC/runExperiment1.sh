#!/bin/bash
### Tracking PID
python3.8 ./2DQuadcopter-RBC-PID.py --overrides ./config_pid.yaml
## Tracking MPC
#python3.8 ./2DQuadcopter-RBC-MPC.py --overrides ./config_mpc.yaml
