#!/bin/bash
bash .vscode/scripts/build.sh

source install/setup.bash
ros2 launch maciv2 gazebo.launch.py

