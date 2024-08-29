#!/bin/bash
bash .vscode/scripts/build.sh

source install/setup.bash
ros2 launch my_first_robot gazebo_empty.launch.py
