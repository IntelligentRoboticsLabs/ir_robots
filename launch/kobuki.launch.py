# Copyright (c) 2023 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from ament_index_python.packages import (get_package_share_directory,
                                         get_package_prefix)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += os.pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    return model_paths


def generate_launch_description():

    robots_dir = get_package_share_directory('ir_robots')

    config = os.path.join(robots_dir, 'config', 'params.yaml')

    params_file = os.path.join(robots_dir, 'params', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        kobuki_params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    ld = LaunchDescription()

    kobuki_cmd = Node(package='kobuki_node',
                      executable='kobuki_ros_node',
                      output='screen',
                      parameters=[kobuki_params],
                      remappings=[
                        ('/commands/velocity', '/cmd_vel'),
                      ])

    ld.add_action(kobuki_cmd)

    kobuki_camera = conf['ir_robots']['kobuki_camera']
    kobuki_lidar = conf['ir_robots']['kobuki_lidar']

    if 'xtion' in kobuki_camera:
        xtion_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('openni2_camera'),
                'launch/'), 'camera_with_cloud.launch.py']),)

        robot_description_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('kobuki_description'),
                'launch/'), 'kobuki_xtion_description.launch.py']),)

        ld.add_action(robot_description_cmd)
        ld.add_action(xtion_cmd)

    elif 'astra' in kobuki_camera:
        astra_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('astra_camera'),
                'launch/'), 'astra_mini.launch.py']),)

        robot_description_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('kobuki_description'),
                'launch/'), 'kobuki_astra_description.launch.py']),)

        ld.add_action(robot_description_cmd)
        ld.add_action(astra_cmd)

    else:
        robot_description_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('kobuki_description'),
                'launch/'), 'kobuki_description.launch.py']),)

        ld.add_action(robot_description_cmd)
        print("NO CAMERA")

    if kobuki_lidar:
        rplidar_cmd = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': True,
                'angle_compensate': True,
            }],)

        laser_filter_cmd = Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    robots_dir,
                    "params", "footprint_filter.yaml",
                ])],)

        ld.add_action(rplidar_cmd)
        ld.add_action(laser_filter_cmd)

    else:
        print("NO LIDAR")

    packages = ['kobuki_description']
    model_path = get_model_paths(packages)

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))

    tf_footprint2base_cmd = Node(package='tf2_ros',
                                 executable='static_transform_publisher',
                                 output='screen',
                                 arguments=['0.0', '0.0', '0.01',
                                            '-1.56', '0.0', '-1.56',
                                            'base_link',
                                            'base_footprint'])

    ld.add_action(tf_footprint2base_cmd)
    return ld
