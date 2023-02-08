#!/bin/bash

# COLCON_IGNORE astra camera
touch ../ThirdParty/ros_astra_camera/astra_camera/COLCON_IGNORE
touch ../ThirdParty/ros_astra_camera/astra_camera_msgs/COLCON_IGNORE
touch ../ThirdParty/kobuki_ftdi/COLCON_IGNORE

# Building project
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# Setup Gazebo to find models - GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.bash
if ! grep -q "source /usr/share/gazebo/setup.bash" ~/.bashrc; then echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc; fi

# Project's path
source install/setup.bash
if ! grep -q "source "${PWD}"/install/setup.bash" ~/.bashrc; then echo "source "${PWD}"/install/setup.bash" >> ~/.bashrc; fi

# Setup CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if ! grep -q "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" ~/.bashrc; then echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc; fi
