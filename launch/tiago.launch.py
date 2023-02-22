# Copyright (c) 2023 José Miguel Guerrero Hernández
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    robots_dir = get_package_share_directory('ir_robots')

    config = os.path.join(robots_dir, 'config', 'params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    simulation = conf['ir_robots']['simulation']

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
                robots_dir, 'launch', 'dependencies'), '/tiago_navigation_sim.launch.py']),
    )

    if not simulation:
        nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                robots_dir, 'launch', 'dependencies'), '/tiago_navigation_real.launch.py']),
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(gz_pose)
    ld.add_action(nav)

    return ld
