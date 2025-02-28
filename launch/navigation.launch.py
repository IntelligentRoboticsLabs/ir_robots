# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Get the launch directory
    robots_dir = get_package_share_directory('ir_robots')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    config = os.path.join(robots_dir, 'config', 'params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robots_dir, 'maps',
                                   conf['ir_robots']['world']+'.yaml'),
        description='Full path to map yaml file to load')

    simulation = conf['ir_robots']['simulation']

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robots_dir, 'params',
                                   conf['ir_robots']['robot']+'_nav_params_sim.yaml'),
        description='Full path to the ROS2 parameters file to \
                    use for all launched nodes')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value=str(conf['ir_robots']['slam']),
        description='Slam'
    )

    if not simulation:
        declare_params_file_cmd = DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(robots_dir, 'params',
                                       conf['ir_robots']['robot']+'_nav_params_real.yaml'),
            description='Full path to the ROS2 parameters file to \
                        use for all launched nodes')

        declare_use_sim_time_cmd = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. \
                    Applied when composition \
                    is disabled.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir,
                                   'rviz',
                                   'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                   'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                   'slam_launch.py')),
        condition=IfCondition(slam),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'use_respawn': use_respawn,
                          'params_file': params_file}.items())

    localization_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                                            launch_dir,
                                            'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={
                'namespace': namespace,
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_composition': use_composition,
                'use_respawn': use_respawn,
                'container_name': 'nav2_container'
            }.items())

    nav_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robots_dir, 'launch', 'dependencies',
                             'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'params_file': params_file,
                'use_lifecycle_mgr': 'false',
                'map_subscribe_transient_local': 'true'
            }.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(rviz_cmd)
    ld.add_action(nav_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(localization_cmd)

    return ld
