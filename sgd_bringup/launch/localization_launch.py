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

    #map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params')

    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_sim_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Provide the transform between earth and map frame
    # this is the transformation in WGS84 coordinates to map origin specified in <map>.yaml
    start_tf_earth_map_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        #arguments=["10.0192452", "53.5532264", "0", "0", "0", "0", "earth", "map"]) # Lohmuehlenpark (alt)
        arguments=["10.0181853", "53.5525418", "0", "0", "0", "0", "earth", "map"]) # Lohmuehlenpark (neu)
        #arguments=["9.9163604", "53.5436909", "0", "0", "0", "0", "earth", "map"])   # Test im Augustinum

    start_sgd_map_server_cmd = Node(
        package='sgd_map_server',
        executable='sgd_map_server',
        name='sgd_map_server',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_gps_cmd = Node(
        package="gnss",
        executable="gnss_node",
        name="gnss_node",
        output="screen",
        emulate_tty=True,
        parameters=[params])

    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_odom_ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        emulate_tty=True,
        parameters=[{os.path.join(bringup_dir, 'config', 'dual_ekf.yaml')},
                        {'use_sim_time': use_sim_time}])

    start_map_ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        emulate_tty=True,
        parameters=[{os.path.join(bringup_dir, 'config', 'dual_ekf.yaml')},
                    {'use_sim_time': use_sim_time}])

    start_uwb_cmd = Node(
        package='uwb',
        executable='uwb_node',
        name='uwb_node',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_scan_matcher_cmd = Node(
        package='sgd_localization',
        executable='scan_matcher',
        name='scan_matcher',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    # Visualizer for debugging
    start_visualizer_cmd = Node(
        package='sgd_util',
        executable='visualizer',
        name='visualizer',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_sim_cmd)

    # nodes publishing transforms
    ld.add_action(start_tf_earth_map_cmd)
    ld.add_action(start_odom_ekf_cmd)
    ld.add_action(start_map_ekf_cmd)
    ld.add_action(start_gps_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_sgd_map_server_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_uwb_cmd)
    ld.add_action(start_scan_matcher_cmd)
    ld.add_action(start_visualizer_cmd)

    return ld
