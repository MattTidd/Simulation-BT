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
    pkg_path = os.path.join(get_package_share_directory('sim_robot_description'))
    joy_params_path = os.path.join(pkg_path, 'config', 'params_joy.yaml')
    gazebo_params_path = os.path.join(pkg_path, 'config', 'params_gazebo.yaml')
    xacro_path = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_path])
    rsp_params = {'robot_description': ParameterValue(robot_description_config, value_type = str), 'use_sim_time': True}

    # launch arguments:
    world = PathJoinSubstitution([pkg_path, 'worlds', LaunchConfiguration('world_name')])
    use_teleop = LaunchConfiguration('use_teleop')

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value = 'empty.world',
        description = 'Name of the world to be launched, within the Worlds folder'
    )

    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value = 'false',
        description = 'Use teleoperation if true'
    )

    # logging arguments:
    arg_log = LogInfo(
        condition = None,
        msg = ['\nworld_name: ', LaunchConfiguration('world_name'),
               ' | use_teleop: ', LaunchConfiguration('use_teleop')]
    )

    # nodes:
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [rsp_params]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments = {
                        # 'verbose': 'true',
                        'world': world,
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_path}
                        .items()
             )
    
    spawn_entity = Node(package = 'gazebo_ros', executable = 'spawn_entity.py',
                        arguments = ['-topic', 'robot_description',
                                   '-entity', 'X3',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-Y', '0.0'],
                        output = 'screen',
                    )

    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        name = 'joy_node',
        parameters = [joy_params_path],
        condition = IfCondition(use_teleop)
    )

    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable = 'teleop_node',
        name = 'teleop_node',
        parameters = [joy_params_path],
        condition = IfCondition(use_teleop)
    )

    return LaunchDescription([
        world_arg,
        use_teleop_arg,
        arg_log,

        robot_state_publisher,
        gazebo, 
        spawn_entity,
        
        joy_node, 
        teleop_node
    ])
    
