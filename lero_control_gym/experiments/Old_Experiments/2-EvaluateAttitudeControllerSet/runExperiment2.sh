#!/bin/bash
### Tracking PID
python3.8 ./Experiment2-Nominal-Condition.py --overrides ./Experiment2-Nominal-Condition.yaml --use_gpu
python3.8 ./Experiment2-RotorLOE-Condition.py --overrides ./Experiment2-RotorLOE-Condition.yaml --use_gpu