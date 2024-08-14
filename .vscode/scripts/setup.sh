#!/bin/bash
set -e
rosdep init 
sudo apt-get update
rosdep update --rosdistro=jazzy
rosdep install --from-paths src --ignore-src -y -r --rosdistro=jazzy