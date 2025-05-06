# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    remapping = [('odometry/filtered', 'odom')]

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_local',
            output='screen',
            remappings=remapping,
            parameters=[os.path.join(get_package_share_directory("nav2_bringup"), 'params', 'nav2_params_real.yaml')],
           ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_global',
            output='screen',
            remappings=remapping,
            parameters=[os.path.join(get_package_share_directory("nav2_bringup"), 'params', 'nav2_params_real.yaml')],
           ),
    ])
