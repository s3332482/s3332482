FROM osrf/ros:humble-desktop
WORKDIR /workspace
COPY . /workspace/.devcontainer/
RUN apt-get update \
    && apt-get install -y ros-humble-ros-gz  ros-humble-rqt-robot-steering ros-humble-slam-toolbox ros-humble-navigation2 \
    ros-humble-nav2-bringup wget python3-pip vim ros-humble-ros2-controllers ros-humble-realsense2-description \
    ros-humble-rqt-robot-steering ros-humble-ign-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control-demos \
    ros-humble-moveit-planners-chomp ros-humble-moveit-ros-perception ros-humble-ur-description ros-humble-rclpy ros-humble-action-msgs \
    ros-humble-control-msgs ros-humble-geometry-msgs ros-humble-moveit-msgs ros-humble-sensor-msgs ros-humble-std-srvs ros-humble-trajectory-msgs \
    ros-humble-launch-param-builder \
    && rm -rf /var/lib/apt/lists/*
RUN bash .devcontainer/setup_container.bash
