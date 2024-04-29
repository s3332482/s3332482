#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

chmod +x /home/ros/apps/FreeCAD.AppImage
echo "Starting FreeCAD, please wait ..."
/home/ros/apps/FreeCAD.AppImage --appimage-extract-and-run