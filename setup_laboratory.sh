#!/bin/bash
RED=`tput setaf 9 blink`

# Installing ThirdParty repos
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y

OUTPUT=$(vcs import < thirdparty.repos | tee >(head -n 1) | tee >(cat >&2) | head -n 1)
if echo "$OUTPUT" | grep -q "E"; then
  echo -e "\n${RED}Error downloading thridparty repos: please check the URLs or connection and re-run this script\n\033[0;33m"
  exit 1
fi

# COLCON_IGNORE astra camera
touch ../ThirdParty/ros_astra_camera/astra_camera/COLCON_IGNORE
touch ../ThirdParty/ros_astra_camera/astra_camera_msgs/COLCON_IGNORE
touch ../ThirdParty/kobuki_ftdi/COLCON_IGNORE

# Building project
cd ../..
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
