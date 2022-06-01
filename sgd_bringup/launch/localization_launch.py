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

    # Create our own temporary YAML files that include substitutions
    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'yaml_filename': map_yaml_file,
    #     'tag_defs': os.path.join(bringup_dir, 'config', 'uwb_tag_defs.yaml'),
    #     'parser_file': os.path.join(bringup_dir, 'config', 'nmea.xml')}

    # configured_params = RewrittenYaml(
    #     source_file=params,
    #     param_rewrites=param_substitutions,
    #     convert_types=True)

    # config_file = os.path.join(bringup_dir, 'config', 'dual_ekf.yaml')

    # Set env var to print messages to stdout immediately
    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # declare_map_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(bringup_dir, 'maps', 'lohmuehlenpark.yaml'),
    #     description='Full path to map yaml file to load')

    declare_sim_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # declare_params_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use')

    # Provide the transform between earth and map frame
    # this is the transformation in WGS84 coordinates to map origin specified in <map>.yaml
    start_tf_earth_map_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["10.0192452", "53.5532264", "0", "0", "0", "0", "earth", "map"]) # Lohmuehlenpark
        #arguments=["9.916281", "53.543635", "0", "0", "0", "0", "earth", "map"])   # Test im Augustinum

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params])

    start_gps_cmd = Node(
        package="gps",
        executable="navilock_ublox6_gps",
        name="ublox6_gps",
        output="screen",
        emulate_tty=True,
        parameters=[params])

    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params])

    start_odom_ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[{os.path.join(bringup_dir, 'config', 'dual_ekf.yaml')},
                        {'use_sim_time': use_sim_time}])

    start_map_ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[{os.path.join(bringup_dir, 'config', 'dual_ekf.yaml')},
                    {'use_sim_time': use_sim_time}])

    start_uwb_cmd = Node(
        package='uwb',
        executable='uwb_node',
        name='uwb_node',
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
    #ld.add_action(declare_map_cmd)
    #ld.add_action(declare_params_cmd)

    # nodes publishing transforms
    ld.add_action(start_tf_earth_map_cmd)
    ld.add_action(start_odom_ekf_cmd)
    ld.add_action(start_map_ekf_cmd)
    ld.add_action(start_gps_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_uwb_cmd)
    ld.add_action(start_visualizer_cmd)

    return ld
