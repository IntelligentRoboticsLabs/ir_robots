# Robots from the Intelligent Robotics Lab using ROS 2

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![humble-devel](https://github.com/IntelligentRoboticsLabs/ir_robots/actions/workflows/master.yaml/badge.svg?branch=ros2-humble)](https://github.com/IntelligentRoboticsLabs/ir_robots/actions/workflows/master.yaml)

This project contains the launchers to run the Tiago robot from [PAL Robotics](https://github.com/pal-robotics) and [Turtlebot2 Kobuki](https://github.com/kobuki-base), both in simulated running different Gazebo worlds, including the [AWS Robomaker](https://github.com/aws-robotics) worlds, as in the real robot using its drivers.

**Recommended: use [Eclipse Cyclone DDS](https://docs.ros.org/en/foxy/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html). 
You can do this by installing it with `sudo apt install ros-humble-rmw-cyclonedds-cpp` and setting the `RMW_IMPLEMENTATION` environment variable: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. Add it to your `.bashrc`**

# Installation on your own computer
You need to have previously installed ROS2. Please follow this [guide](https://docs.ros.org/en/humble/Installation.html) if you don't have it.
```bash
source /opt/ros/humble/setup.bash
```

Clone the repository to your workspace:
```bash
cd <ros2-workspace>/src
git clone https://github.com/IntelligentRoboticsLabs/ir_robots.git
```

Prepare your thirparty repos:
```bash
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
cd <ros2-workspace>/src/
vcs import < ir_robots/thirdparty.repos
```
*Please make sure that this last command has not failed. If this happens, run it again.*

### Install libusb, libftdi & libuvc
```bash
sudo apt install libusb-1.0-0-dev libftdi1-dev libuvc-dev
```

### Install udev rules from astra camera, kobuki and rplidar
When you connect a piece of hardware to your pc, it assigns `/dev/ttyUSB*` to it. This will not have the necessary read/write permissions, so we will not be able to use it correctly. The solution is to set up some udev rules that creates a symlink with another name (example: `/dev/ttyUSB0` -> `/dev/kobuki`) and grants it the necessary permissions.
```bash
cd <workspace-ros2>
sudo cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/kobuki_ftdi/60-kobuki.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Move xtion calibration
Some cameras need a calibration file where they indicate, for example, their resolution, name, etc...
```bash
mkdir -p ~/.ros/camera_info
cp <ros2-workspace>/src/ThirdParty/openni2_camera/openni2_camera/rgb_PS1080_PrimeSense.yaml ~/.ros/camera_info
```

### Building project
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```
*For this build we use `--cmake-args -DBUILD_TESTING=OFF` to avoid compiling our tests as well and save time. It is recommended to compile later without this flag.*


### Setup Gazebo to find models - GAZEBO_MODEL_PATH and project path
```bash
source /usr/share/gazebo/setup.bash
source <ros2-workspace>/install/setup.bash
```
*It is recommended to add these two lines inside your `.bashrc` to avoid having to run it every time you open a new shell*

# Installation in the laboratories of the university
In the computers of the university laboratories you will already have all the ros 2 packages installed, so you will not have to install any tools or dependencies. Add `COLCON_IGNORE` to the packages used by real robots to avoid any problems when building.

```bash
cd <ros2-workspace>/src/
git clone https://github.com/IntelligentRoboticsLabs/ir_robots.git
vcs import < ir_robots/thirdparty.repos

touch ThirdParty/ros_astra_camera/astra_camera/COLCON_IGNORE
touch ThirdParty/ros_astra_camera/astra_camera_msgs/COLCON_IGNORE
touch ThirdParty/kobuki_ros/kobuki_node/COLCON_IGNORE
touch ThirdParty/kobuki_ros/kobuki_auto_docking/COLCON_IGNORE
touch ThirdParty/kobuki_core/COLCON_IGNORE
touch ThirdParty/kobuki_ftdi/COLCON_IGNORE
touch ThirdParty/ecl/ecl_core/COLCON_IGNORE
touch ThirdParty/ecl/ecl_lite/COLCON_IGNORE
touch ThirdParty/openni2_camera/openni2_camera_msgs/COLCON_IGNORE
touch ThirdParty/openni2_camera/openni2_camera/COLCON_IGNORE

source /opt/ros/<ros2-distro>/setup.bash
cd <ros2-workspace>
colcon build --symlink-install
```
*Remember that in the laboratories you will only be able to run the simulation of the environment with the different robots, you will not be able to use it on a real robot.*

# Run Gazebo & robot in ROS2

Modify `config/params.yaml` to select the robot (kobuki/tiago), select the world and starting positions:
```yaml
...
ir_robots:
  simulation: true
  world: aws_house
  robot: kobuki
  robot_position:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  tiago_arm: no-arm
  kobuki_camera: none
  kobuki_lidar: false
...
```

Then, launch your simulation environment:
```bash
source install/setup.sh
ros2 launch ir_robots simulation.launch.py
``` 

If you have a low performance, close the Gazebo's client. Check gzclient process, and kill it:
```bash
kill -9 `pgrep -f gzclient`
``` 

# Run a real kobuki in ROS 2

First, modify `config/params.yaml` to use kobuki, if you are using camara (xtion/astra/none) and if you are using lidar (true/false)
```yaml
...
ir_robots:
  simulation: true
  world: aws_house
  robot: kobuki
  robot_position:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  kobuki_camera: astra
  kobuki_lidar: true
...
```

Then, run the kobuki drivers:
```bash
source install/setup.sh
ros2 launch ir_robots kobuki.launch.py
``` 

# Run Navigation in ROS 2

You can use [Nav2] using robot in the selected world in your `config/params.yaml`:

```bash
source install/setup.sh
ros2 launch ir_robots navigation.launch.py
``` 
If simulation param is set to `false`, Navigation2 is ready to use in the real robot.

Also, you can use [Keepout Zones], just create a new map including the excluded areas, and use the same name adding `_keep`, now publish the map running:

```bash
source install/setup.sh
ros2 launch ir_robots keepzone.launch.py
``` 

Just some AWS worlds are included.

# Run the SLAM
## Launch SLAM
If you want to make your own map, set slam parameter to true in the `config/params.yaml`:

```bash
source install/setup.sh
ros2 launch ir_robots navigation.launch.py
``` 

## Save the map
```bash
ros2 run nav2_map_server map_saver_cli --ros-args -p use_sim_time:=true
```

## Load the new map
Move the map to the `maps/` folder inside the package. Then remember to rename it and modify the name inside the yaml. Finally, modify the world parameter, adding the name of your new map.

# About

This is a project made by the [Intelligent Robotics Lab], a research group from the [Universidad Rey Juan Carlos].
Copyright &copy; 2023.

Maintainers:

* [José Miguel Guerrero]
* [Juan Carlos Manzanares]
* [Francisco Martín]


## License

Shield: 

[![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

This work is licensed under a
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg


[Universidad Rey Juan Carlos]: https://www.urjc.es/
[Intelligent Robotics Lab]: https://intelligentroboticslab.gsyc.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
[Juan Carlos Manzanares]: https://github.com/Juancams
[Francisco Martín]: https://github.com/fmrico
[Nav2]: https://navigation.ros.org/
[Keepout Zones]: https://navigation.ros.org/tutorials/docs/navigation2_with_keepout_filter.html?highlight=keep
[SLAM Toolbox]: https://vimeo.com/378682207
[Navigate While Mapping]: https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html
