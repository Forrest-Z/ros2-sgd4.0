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
    params_file = LaunchConfiguration('params_file')
    
    gps_port = LaunchConfiguration('gps_port')
    feather_port = LaunchConfiguration('feather_port')
    esp_port = LaunchConfiguration('esp_port')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'gps_port': '/dev/ttyACM0',
        'esp_port': '/dev/ttyUSB0',
        'feather_port': '/dev/ttyACM0'}

    configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True)

        # Set env var to print messages to stdout immediately
        #SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
 
    declare_sim_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'sgd_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_gps_port_cmd = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyACM0',
        description='Port to communication with gps sensor')

    declare_feather_port_cmd = DeclareLaunchArgument(
        'feather_port',
        default_value='/dev/ttyACM0',
        description='Port to communicate with feather M0')

    declare_esp_port_cmd = DeclareLaunchArgument(
        'esp_port',
        default_value='/dev/ttyUSB0',
        description='Port to communicate with ESP8266')

    start_gps_cmd = Node(
        package="gps",
        executable="navilock_ublox6_gps",
        name="ublox6_gps",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": gps_port,
            "xml_file": '/home/ipp/dev_ws/src/ros2-sgd4.0/sensors/gps/data/nmea.xml'}])

    start_sick_lidar_cmd = Node(
        package='sick_scan2',
        name = 'sick_scan2',
        executable='sick_generic_caller',
        output='screen',
        parameters = [configured_params])

    start_receiver_cmd = Node(
        package='frsky_rx8r',   
        name='frsky_rx8r',
        executable='frsky_rx8r',
        parameters=[{
            'port': '/dev/ttyACM0'
    }])

    start_led_strip_cmd = Node(
        package='led_strips',
        name='led_strip',
        executable='led_strip',
        parameters=[{
            'port': '/dev/ttyACM0'
    }])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_sim_cmd)
    ld.add_action(declare_params_cmd)
    ld.add_action(declare_gps_port_cmd)
    ld.add_action(declare_feather_port_cmd)
    ld.add_action(declare_esp_port_cmd)

    # Add the actions to launch all of the navigation nodes
    #ld.add_action(start_gps_cmd)
    #ld.add_action(start_sick_lidar_cmd)
    #ld.add_action(start_led_strip_cmd)
    ld.add_action(start_receiver_cmd)

    return ld