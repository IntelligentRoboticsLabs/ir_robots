# Copyright 2023 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robots_dir = get_package_share_directory('ir_robots')
    kobuki_dir = get_package_share_directory('kobuki_description')

    urdf_file = os.path.join(kobuki_dir, 'urdf', 'kobuki_gazebo.urdf')

    with open(urdf_file, 'r') as info:
        robot_desc = info.read()

    config = os.path.join(robots_dir, 'config', 'params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    model_name = DeclareLaunchArgument(
        'model_name', default_value='kobuki',
        description='Gazebo model name'
    )

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

    kobuki_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                         arguments=['-topic', 'robot_description',
                                    '-entity', LaunchConfiguration('model_name'),
                                    ' '.join(['-x', str(conf['ir_robots']
                                                            ['robot_position']['x'])]),
                                    ' '.join(['-y', str(conf['ir_robots']
                                                            ['robot_position']['y'])]),
                                    ' '.join(['-z', str(conf['ir_robots']
                                                            ['robot_position']['z'])]),
                                    ' '.join(['-R', str(conf['ir_robots']
                                                            ['robot_position']['roll'])]),
                                    ' '.join(['-P', str(conf['ir_robots']
                                                            ['robot_position']['pitch'])]),
                                    ' '.join(['-Y', str(conf['ir_robots']
                                                            ['robot_position']['yaw'])]),
                                    # LaunchConfiguration('gzpose'),
                                    ],
                         output='screen')

    tf_footprint2base_cmd = Node(package='tf2_ros', executable='static_transform_publisher',
                                 output='screen',
                                 arguments=['0.0', '0.0', '0.01',
                                            '-1.56', '0.0', '-1.56',
                                            'base_link',
                                            'base_footprint'])

    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(gz_pose)
    ld.add_action(model_name)
    ld.add_action(robot_model)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(kobuki_entity)
    ld.add_action(tf_footprint2base_cmd)

    return ld
