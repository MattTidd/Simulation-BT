# Copyright (c) 2024 Matthew Allan Tidd
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

This launch file simply launches a robot within Gazebo and allows for basic 
teleoperation using ROS2_Control.

"""

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import numpy as np

def generate_launch_description():
    # paths & params:
    pkg_path = os.path.join(get_package_share_directory('sim_robot_slam'))
    config_path = os.path.join(pkg_path, 'config', 'params_slam.yaml')

    # nodes:
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
            launch_arguments = {'slam_params_file': config_path,
                                'use_sim_time' : 'true'}.items()
        )
    
    return LaunchDescription([
        slam_node
    ])

