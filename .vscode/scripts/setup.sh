#!/bin/bash
set -e
rosdep init 
sudo apt-get update
rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -y --rosdistro=humble