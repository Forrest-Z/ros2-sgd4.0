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

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    #remappings = [('/tf', 'tf'),
    #              ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    # param_substitutions = {
    #     'use_sim_time': use_sim_time,
    #     'default_bt_xml_filename': default_bt_xml_filename,
    #     'map_subscribe_transient_local': map_subscribe_transient_local,
    #     'yaml_filename': map_yaml_file,
    #     'log_dir': '~home/.ros/log/'}

    # configured_params = RewrittenYaml(
    #         source_file=params_file,
    #         param_rewrites=param_substitutions,
    #         convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # DeclareLaunchArgument(
        #     'params',
        #     default_value=os.path.join(bringup_dir, 'params', 'simulation_params.yaml'),
        #     description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                bringup_dir,'behavior_trees', 'navigate_w_replanning_and_recovery_sgd.xml'),
            description='Full path to the behavior tree xml file to use'),

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'),

        Node(
            package='sgd_controller',
            executable='subsum_controller',
            name='subsum_controller',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='sgd_safety',
            executable='ros2_obstacle_checker',
            name='ros2_obstacle_checker',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='sgd_safety',
            executable='ros2_depthCamera_obstacle_checker',
            name='ros2_depthCamera_obstacle_checker',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='sgd_global_planner',
            executable='global_planner',
            name='osm_planner',
            output='screen',
            emulate_tty=True,
            parameters=[params]),

        Node(
            package='frsky_rx8r',
            executable='frsky_rx8r',
            name='frsky_rx8r',
            output='screen',
            parameters=[params]),

        Node(
            package='sgd_local_costmap',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            emulate_tty=True,
            parameters=[params])
    ])
