#!/bin/bash
### Tracking PID
python3.8 ./Experiment1-PSO-NominalConditions.py --overrides ./Experiment1-PSO-NominalConditions.yaml --use_gpu
python3.8 ./Experiment1-PSO-RotorLOECondition.py --overrides ./Experiment1-PSO-RotorLOECondition.yaml --use_gpu

