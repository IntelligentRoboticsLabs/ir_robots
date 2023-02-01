#!/bin/bash

# Installing ThirdParty repos
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
vcs import < thirdparty.repos

# Prepare other repos
cd ../..
mkdir repos
cd repos

# Install glog
wget -c https://github.com/google/glog/archive/refs/tags/v0.6.0.tar.gz  -O glog-0.6.0.tar.gz
tar -xzvf glog-0.6.0.tar.gz
rm -rf glog-0.6.0.tar.gz
cd glog-0.6.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
cd ..
touch COLCON_IGNORE
cd ..

# Install magic_enum
wget -c https://github.com/Neargye/magic_enum/archive/refs/tags/v0.8.0.tar.gz -O  magic_enum-0.8.0.tar.gz
tar -xzvf magic_enum-0.8.0.tar.gz
rm -rf magic_enum-0.8.0.tar.gz
cd magic_enum-0.8.0
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
cd ..
touch COLCON_IGNORE
cd ..

# Install libuvc
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig
cd ..
touch COLCON_IGNORE
cd ../..

# Install libusb rules from astra camera, kobuki and rplidar
sudo cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/kobuki_ftdi/60-kobuki.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Move xtion calibration
mkdir -p ~/.ros/camera_info
cp src/ThirdParty/openni2_camera/openni2_camera/rgb_PS1080_PrimeSense.yaml ~/.ros/camera_info

# Move kobuki model to GAZEBO_MODEL_PATH
mkdir -p ~/.gazebo/models/kobuki_description
cp -r src/ThirdParty/kobuki_ros/kobuki_description/meshes ~/.gazebo/models/kobuki_description/meshes

# Building project
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

# Setup Gazebo to find models - GAZEBO_MODEL_PATH
source /usr/share/gazebo/setup.bash
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc

# Project's path
source install/setup.bash
echo "source "${PWD}"/install/setup.bash" >> ~/.bashrc
