#!/bin/bash

sudo apt-get update 

sudo apt-get install -y ros-jazzy-ros-gz  ros-jazzy-rqt-robot-steering ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup wget python3-pip vim ros-jazzy-ros2-controllers ros-jazzy-realsense2-description \
    ros-jazzy-rqt-robot-steering ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control-demos \
    ros-jazzy-moveit-planners-chomp ros-jazzy-moveit-ros-perception ros-jazzy-ur-description ros-jazzy-rclpy ros-jazzy-action-msgs \
    ros-jazzy-control-msgs ros-jazzy-geometry-msgs ros-jazzy-moveit-msgs ros-jazzy-sensor-msgs ros-jazzy-std-srvs ros-jazzy-trajectory-msgs \
    ros-jazzy-launch-param-builder ros-jazzy-moveit-setup-assistant ros-jazzy-moveit-configs-utils ros-jazzy-moveit-simple-controller-manager\
    ros-jazzy-moveit-planners-ompl ros-jazzy-pilz-industrial-motion-planner ros-jazzy-moveit-planners-stomp


wget -O ~/FreeCAD.AppImage https://github.com/FreeCAD/FreeCAD-Bundle/releases/download/weekly-builds/FreeCAD_weekly-builds-38459-conda-Linux-aarch64-py311.AppImage
chmod +x ~/FreeCAD.AppImage 

# Setup FreeCAD Cross For FreeCAD
 mkdir -p ~/.local/share/FreeCAD/Mod/
git clone https://github.com/johnny555/freecad.cross.git ~/.local/share/FreeCAD/Mod/freecad.cross

# Get python deps
pip3 install black urdf-parser-py

# Assuming

# Make it so that sourcing happens automatically
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/bar_ws/install/setup.bash" >> ~/.bashrc

# Add GAZEBO path so we can easily include models. Must use merge install. 
# Sholud be able to use package:// syntax now. 
echo "export GZ_SIM_RESOURCE_PATH=~/bar_ws/install/share/:/opt/ros/jazzy/share/" >> ~/.bashrc
echo "export SDF_PATH=~/bar_ws/install/share/:/opt/ros/jazzy/share/" >> ~/.bashrc