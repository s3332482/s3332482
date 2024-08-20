#!/bin/bash
bash .vscode/scripts/build.sh

source install/setup.bash
echo $SDF_PATH
ros2 launch krytn gazebo.launch.py

