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

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    
    gps_port = LaunchConfiguration('gps_port')
    feather_port = LaunchConfiguration('feather_port')
    esp_port = LaunchConfiguration('esp_port')

    lifecycle_nodes = [#'gps_serial',
                       #'ublox6_gps',
                       'esp_serial',
                       'imu_bno055',
                       'wh_fcruiser',
                       'feather_serial']
    #                   'capacitive_touch',
    #                   'laser_1d',

   # lifecycle_nodes = ['esp_serial',
   #                    'imu_bno055']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'gps_port': '/dev/ttyACM0',
        'esp_port': '/dev/ttyUSB0',
        'feather_port': '/dev/ttyACM0'}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    config = os.path.join(
        bringup_dir,
        'params',
        'sick_tim_5xx.yaml'
        )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
 
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
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

        Node(
            package='sgd_util',
            executable='logger',
            name='logger',
            output='screen',
            parameters=[{'output_folder': os.path.join('/home/ipp/dev_ws','log')}]),
                     
        # TODO: Create nodes for lidar
        Node(
            package='sick_scan2',
            name = 'sick_scan2',
            executable='sick_generic_caller',
            output='screen',
            parameters = [config]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_hardware',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}]),
    ])
