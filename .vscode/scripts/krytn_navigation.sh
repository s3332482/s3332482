#!/bin/bash
bash .vscode/scripts/build.sh

source install/setup.bash
export SDF_PATH=/workspace/install/krytn/share:/workspace/install/krytn/:/opt/ros/jazzy/share/
export GZ_SIM_RESOURCE_PATH=$SDF_PATH

ros2 launch krytn navigation.launch.py
