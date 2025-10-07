# Copyright (c) 2025 TODO. All rights reserved.
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
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('interaction_sim'), 'launch'),
        '/simulator.launch.py'])
    ))

    for dep in ["chatbot_ollama",
                "semantic_state_aggregator",
                # "skill_ask_human_for_help",
                "skill_navigate_to_zone_sim",
                "skill_navigate_to_pose_sim",
                "skill_explain",
                "component_explain_planner",
                "component_explain_ask_human_for_help",
                "component_explain_navigation",
                "component_explain_say",
                ]:
        ld.add_action(IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(dep), 'launch'), f'/{dep}.launch.py'])
        ))

    # Add static transforms
    ld.add_action(Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='map_to_base_footprint',
      arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint']
    ))

    ld.add_action(Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='base_footprint_to_base_link',
      arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    ))

    return ld
