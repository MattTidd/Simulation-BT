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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import numpy as np
import os

def generate_launch_description():
    # launch parameters & arguments:
    map_name = LaunchConfiguration('map_name')
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value = 'small_room_map.yaml',
        description = 'Map to be used in navigation'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'false',
        description = 'Whether to use sim time or not, defaults to false'
    )

    print(f'setting paths!')
    pkg_path = get_package_share_directory("sim_bt")
    nav2_params_path = os.path.join(pkg_path, 'params', 'nav2_params.yaml')
    map_file = PathJoinSubstitution([pkg_path, 'maps', map_name])
    print(f'map path is: {map_file}')

    # nodes:
    print(f'instantiating nodes!')
    nav2_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments = {'map' : map_file,
                            'use_sim_time' : use_sim_time,
                            'params_file' : nav2_params_path}.items()
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

    print(f'launching!')
    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        
        nav2_bringup,
        bt_node
    ])
