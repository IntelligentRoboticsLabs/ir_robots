#!/bin/bash

# COLCON_IGNORE astra camera
touch ../ThirdParty/ros_astra_camera/astra_camera/COLCON_IGNORE
touch ../ThirdParty/ros_astra_camera/astra_camera_msgs/COLCON_IGNORE
touch ../ThirdParty/kobuki_ftdi/COLCON_IGNORE

# Move kobuki model to GAZEBO_MODEL_PATH
cd ../..
mkdir -p ~/.gazebo/models/kobuki_description
cp -r src/ThirdParty/kobuki_ros/kobuki_description/meshes ~/.gazebo/models/kobuki_description/meshes

# Building project
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# Setup Gazebo to find models - GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc

# Project's path
source install/setup.bash
echo "source "${PWD}"/install/setup.bash" >> ~/.bashrc
