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
    sgd_comm_dir = get_package_share_directory('sgd_comm')

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

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
 
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'sgd_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        DeclareLaunchArgument(
	        'gps_port',
	        default_value='/dev/ttyACM0',
	        description='Port to communication with gps sensor'),

    	DeclareLaunchArgument(
	        'feather_port',
	        default_value='/dev/ttyACM0',
	        description='Port to communicate with feather M0'),

        DeclareLaunchArgument(
	        'esp_port',
	        default_value='/dev/ttyUSB0',
	        description='Port to communicate with ESP8266'),

        Node(
            package="sgd_comm",
            executable="serial",
            name="gps_serial",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": gps_port,
                 "baud_rate": 115200,
                 "read_write": "ro",
                 "logfile": os.path.join('/home/ipp/dev_ws','log','serial_gps.log'),
                 "raw": True,
                 "sframe": '\n',
                 "stframe": '\n',
                 "log": False}]),

        Node(
            package="gps",
            executable="navilock_ublox6_gps",
            name="ublox6_gps",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": gps_port,
                 "xml_file": '/home/ipp/dev_ws/src/ros2-sgd4.0/sensors/gps/data/nmea.xml'}]),

        # Create nodes for capacitive touch and laser 1D
        Node(
            package="sgd_comm",
            executable="serial",
            name="feather_serial",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": feather_port,
                "baud_rate": 115200,
                "read_write": "ro",
                "logfile": os.path.join('/home/ipp/dev_ws','log','serial_feather.log'),
                "raw": False,
                "sframe": '$',
                "stframe": '\n',
                "log": False,
                'use_sim_time': use_sim_time}]),

        Node(
            package='imu',
            executable='imu_bno055',
            name='imu_bno055',
            output='screen',
            parameters=[{'port': esp_port,
                         'config_mode': False,
            		     'use_sim_time': use_sim_time}]),

        Node(
            package="cap_touch",
            executable="capacitive_touch",
            name="capacitive_touch",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": feather_port,
                "thresh": 2000,
                "filter_i": 4}]),

        Node(
            package="laser1d",
            executable="laser_1d",
            name="laser_1d",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": feather_port}]),

        Node(
            package='sgd_comm',
            executable='serial',
            name='esp_serial',
            output='screen',
            parameters=[{'port': esp_port,
            		     'baud_rate': 115200,
            		     'read_write': 'rw',
                         'logfile': os.path.join('/home/ipp/dev_ws','log','serial_esp.log'),
                         'raw': False,
                         'sframe': '$',
                         'stframe': '\n',
                         'log': False,
                         'use_sim_time': use_sim_time}]),

        Node(
            package='motorcontroller',
            executable='wh_fcruiser',
            name='wh_fcruiser',
            output='screen',
            parameters=[{'port': esp_port},
            		 {'motor_kp': 0.1},
            		 {'max_speed': 200.0},
            		 {'use_sim_time': use_sim_time}]),

        # TODO: Create nodes for lidar
        Node(
            package='sick_scan2',
            name = 'sick_scan2',
            executable='sick_generic_caller',
            output='screen',
            parameters = [configured_params])
    ])
