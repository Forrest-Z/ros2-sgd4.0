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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('sgd_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params')

    set_env_rcutils = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_sim_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_bt_xml_filename = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(bringup_dir,'behavior_trees', 'navigate_w_replanning_and_recovery_sgd.xml'),
        description='Full path to the behavior tree xml file to use')

    start_sgd_controller_cmd = Node(
        package='sgd_controller',
        executable='subsum_controller',
        name='subsum_controller',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_obstacle_checker_cmd = Node(
        package='sgd_safety',
        executable='ros2_obstacle_checker',
        name='ros2_obstacle_checker',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_depth_camera_cmd = Node(
        package='sgd_safety',
        executable='ros2_depthCamera_obstacle_checker',
        name='ros2_depthCamera_obstacle_checker',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_nav2_controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_nav2_planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_nav2_recoveries_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        emulate_tty=True,
        on_exit=Node(
            package='sgd_lifecycle_manager',
            executable='exit_publisher',
            parameters=[{'ndname': 'bt_navigator'}]),
        parameters=[params])

    start_waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_global_planner_cmd = Node(
        package='sgd_global_planner',
        executable='global_planner',
        name='osm_planner',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    start_frsky_rx8r_cmd = Node(
        package='frsky_rx8r',
        executable='frsky_rx8r',
        name='frsky_rx8r',
        output='screen',
        parameters=[params])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(set_env_rcutils)
    ld.add_action(declare_sim_cmd)
    ld.add_action(declare_bt_xml_filename)

    ld.add_action(start_sgd_controller_cmd)
    # ld.add_action(start_obstacle_checker_cmd)
    # ld.add_action(start_depth_camera_cmd)
    ld.add_action(start_nav2_controller_cmd)
    ld.add_action(start_nav2_planner_cmd)
    ld.add_action(start_nav2_recoveries_cmd)
    ld.add_action(start_bt_navigator_cmd)
    ld.add_action(start_waypoint_follower_cmd)
    ld.add_action(start_global_planner_cmd)
    ld.add_action(start_frsky_rx8r_cmd)

    return ld