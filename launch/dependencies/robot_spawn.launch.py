# Copyright (c) 2021 PAL Robotics S.L.
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

# Modified by José Miguel Guerrero Hernández
# Modified by Juan Carlos Manzanares Serrano

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_pal.include_utils import include_launch_py_description


def generate_launch_description():
    #    This format doesn't work because we have to expand gzpose into
    #    different args for spawn_entity.py
    #    gz_pose = DeclareLaunchArgument(
    #        'gzpose', default_value='-x 0 -y 0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0 ',
    #        description='Spawn gazebo position as provided to spawn_entity.py'
    #    )

    # @TODO: load PID gains? used in gazebo_ros_control fork
    # @TODO: load tiago_pal_hardware_gazebo

    robots_dir = get_package_share_directory('ir_robots')
    kobuki_dir = get_package_share_directory('kobuki_description')

    urdf_file = os.path.join(kobuki_dir, 'urdf', 'kobuki_gazebo.urdf')

    config = os.path.join(robots_dir, 'config', 'params.yaml')

    with open(urdf_file, 'r') as info:
        robot_desc = info.read()

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    model_name = DeclareLaunchArgument(
        'model_name', default_value='robot',
        description='Gazebo model name'
    )

    tiago_state_publisher = include_launch_py_description(
        'tiago_description',
        ['launch', 'robot_state_publisher.launch.py'])

    # Robot description
    robot_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf_file]
    )

    # TF Tree
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    robot_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity',
                                   LaunchConfiguration('model_name'),
                                   ' '.join(['-x', str(conf['ir_robots']
                                                           ['robot_position']
                                                           ['x'])]),
                                   ' '.join(['-y', str(conf['ir_robots']
                                                           ['robot_position']
                                                           ['y'])]),
                                   ' '.join(['-z', str(conf['ir_robots']
                                                           ['robot_position']
                                                           ['z'])]),
                                   ' '.join(['-R', str(conf['ir_robots']
                                                           ['robot_position']
                                                           ['roll'])]),
                                   ' '.join(['-P', str(conf['ir_robots']
                                                           ['robot_position']
                                                           ['pitch'])]),
                                   ' '.join(['-Y', str(conf['ir_robots']
                                                           ['robot_position']
                                                           ['yaw'])]),
                                   # LaunchConfiguration('gzpose'),
                                   ],
                        output='screen')

    robot = conf['ir_robots']['robot']

    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(gz_pose)
    ld.add_action(model_name)

    if 'kobuki' in robot:
        ld.add_action(joint_state_publisher_node)
        ld.add_action(robot_model)
    elif 'tiago' in robot:
        ld.add_action(tiago_state_publisher)

    ld.add_action(robot_entity)

    return ld
