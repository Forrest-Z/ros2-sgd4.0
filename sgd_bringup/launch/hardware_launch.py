# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('sgd_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params')

    start_sick_lidar_cmd = Node(
        package='sick_scan2',
        name = 'sick_scan2',
        executable='sick_generic_caller',
        output='screen',
        emulate_tty=True,
        parameters = [params])

    start_led_strip_cmd = Node(
        package='led_strips',
        name='led_strip',
        executable='led_strip',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_motorcontroller_cmd = Node(
        package='motorcontroller',
        name='wh_fcruiser',
        executable='motorcontroller',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_feather_handle_cmd = Node(
        package='feather_handle',
        executable='feather_handle_ros',
        name='feather_handle_ros',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_scan_timer_cmd = Node(
        package='sgd_util',
        executable='scan_timer',
        name='scan_timer',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    # Create the launch description and populate
    ld = LaunchDescription()
    #ld.add_action(declare_params_file_cmd)

    # Add the actions to launch all of the navigation nodes
    # ld.add_action(start_sick_lidar_cmd)
    ld.add_action(start_led_strip_cmd)
    ld.add_action(start_motorcontroller_cmd)
    ld.add_action(start_feather_handle_cmd)
    # ld.add_action(start_scan_timer_cmd)

    return ld