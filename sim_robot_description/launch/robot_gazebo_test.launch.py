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
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import numpy as np

def generate_launch_description():
    # paths & params:
    pkg_path = os.path.join(get_package_share_directory('sim_robot_description'))
    joy_params_path = os.path.join(pkg_path, 'config', 'joy_params.yaml')
    gazebo_params_path = os.path.join(pkg_path, 'config', 'gazebo_params.yaml')
    xacro_path = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_path])
    rsp_params = {'robot_description': ParameterValue(robot_description_config, value_type = str), 'use_sim_time': True}
    
    # robot spawns:
    robot_spawns = [(-4.25, 1.75, -np.pi/2), (-4.5, -1.0, np.pi/4), (3.125, 0.75, -np.pi/4),
                    (-1.75, -1.75, np.pi/4), (0.25, -1.75, 3*np.pi/4), (-3.0, 1.75, 0),
                    (-0.5, 1.75, np.pi), (-4.5, 4.5, -np.pi/4), (-4.5, 2.75, np.pi/4),
                    (-3.625, 2.75, 3*np.pi/4), (-2.5, 4.5, -np.pi/2), (-1.625, 4.5, -np.pi/4),
                    (-0.5, 4.5, -3*np.pi/4), (-0.625, 3.125, 3*np.pi/4), (0.375, 1.5, 0),
                    (0.375, 3.0, -np.pi/4), (3.0, 1.0, 3*np.pi/4), (0.5, 4.25, 0),
                    (1.75, 4.5, -np.pi/2), (2.75, 4.25, 0), (4.375, 4.5, -3*np.pi/4),
                    (4.125, 1.75, np.pi/2), (4.125, 0.625, -np.pi/2), (4.25, -1.25, 3*np.pi/4),
                    (3.0, -1.25, np.pi/4), (4.25, -2.5, np.pi), (4.25, -4.0, np.pi),
                    (3.0, -2.25, -np.pi/2), (0.125, -4.5, np.pi/4), (2.0, -4.5, 3*np.pi/4),
                    (0.375, -2.625, -3*np.pi/4), (-1.625, -2.625, -np.pi/4), (-3.0, -4.5, 0),
                    (-4.375, -4.5, np.pi/2), (-4.5, -1.875, -np.pi/4), (-2.625, -1.875, -3*np.pi/4),
                    (0, 0, 0), (3.0, -4.5, np.pi/2)]
    
    # pick a robot spawn:
    x, y, Y = robot_spawns[np.random.randint(0, len(robot_spawns))]

    # launch arguments:
    use_random_spawn = LaunchConfiguration('use_random_spawn')
    world = PathJoinSubstitution([pkg_path, 'worlds', LaunchConfiguration('world_name')])

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value = 'empty.world',
        description = 'Name of the world to be launched, within the Worlds folder'
    )

    use_random_spawn_arg = DeclareLaunchArgument(
        'use_random_spawn',
        default_value = 'false',
        description = 'Spawn the robot in a random position if true'
    )

    # logging arguments:
    arg_log = LogInfo(
        condition = None,
        msg = ['\nworld_name: ', LaunchConfiguration('world_name'),
               ' | use_random_spawn: ', LaunchConfiguration('use_random_spawn')]
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
    
    random_spawn_entity = Node(package = 'gazebo_ros', executable = 'spawn_entity.py',
                        arguments = ['-topic', 'robot_description',
                                   '-entity', 'X3',
                                   '-x', str(x),
                                   '-y', str(y),
                                   '-Y', str(Y)],
                        output = 'screen',
                        condition = IfCondition(use_random_spawn))
    
    spawn_entity = Node(package = 'gazebo_ros', executable = 'spawn_entity.py',
                        arguments = ['-topic', 'robot_description',
                                   '-entity', 'X3',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-Y', '0.0'],
                        output = 'screen',
                        condition = UnlessCondition(use_random_spawn))
    
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        name = 'joy_node',
        parameters = [joy_params_path]
    )

    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable = 'teleop_node',
        name = 'teleop_node',
        parameters = [joy_params_path],
    )

    return LaunchDescription([
        world_arg,
        use_random_spawn_arg,
        arg_log,
        robot_state_publisher,
        gazebo, 
        random_spawn_entity,
        spawn_entity,
        joy_node, 
        teleop_node
    ])
    
