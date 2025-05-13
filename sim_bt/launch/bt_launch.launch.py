# Copyright (c) 2025 Matthew Allan Tidd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This launch file launches the BT executable for a simple inspection task.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

def generate_launch_description():
    # get paths:
    pkg_path = get_package_share_directory("sim_bt")

    # params:
    nav2_params_path = os.path.join(pkg_path, 'params', 'nav2_params.yaml')
    map_file = os.path.join(pkg_path, 'maps', 'small_room_map.yaml')

    # nodes:
    nav2_bringup = Node(
        package = 'nav2_bringup',
        executable = 'bringup_launch.py',
        name = 'nav2_bringup',
        output = 'screen',
        parameters = [nav2_params_path],
        arguments = ['--map', map_file]
    )

    bt_node = Node(
        package = 'sim_bt',
        executable = 'bt_node',
        name = 'simple_inspection_bt',
        output = 'screen',
        parameters = [
            {'tree_file' : get_package_share_directory('sim_bt') + '/trees/my_tree.xml'},
            {'task_locations': [1.14694, 1.751130, -0.706895,
                                -1.833130, -0.640061, -0.815012,
                                -0.117466, 2.513610, 0.791175]},
            {'task_count' : int(3)}]
    )

    return LaunchDescription([
        nav2_bringup,
        bt_node
    ])
