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

def generate_launch_description():
    bt_node = Node(
        package = 'sim_bt',
        executable = 'bt_node',
        name = 'simple_inspection_bt',
        output = 'screen',
        parameters = [
            {'tree_file' : get_package_share_directory('sim_bt') + '/trees/my_tree.xml'},
            {'task_locations': [2.0, 3.0, 1.0]},
            {'task_count' : int(5)}]
    )

    return LaunchDescription([
        bt_node
    ])
