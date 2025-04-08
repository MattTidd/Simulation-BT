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

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # paths & params:
    pkg_path = os.path.join(get_package_share_directory('sim_robot_description'))
    # rviz_config_path = os.path.join(pkg_path, 'config', 'rviz_config.rviz')
    xacro_path = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    robot_description_config = Command(['xacro ', xacro_path])
    rsp_params = {'robot_description': ParameterValue(robot_description_config, value_type = str), 'use_sim_time': False}

    # nodes:
    robot_state_publisher = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [rsp_params]
    )

    joint_state_publisher = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        parameters = [{'use_sim_time': False}]
    )

    rviz = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
        # arguments = ['-d', rviz_config_path]
        parameters = [{'use_sim_time': False}]
    )

    # launch:
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])