import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # input parameters
    #use_simulator = LaunchConfiguration('use_simulator')

    #declare_use_simulator_cmd = DeclareLaunchArgument(
    #    'use_simulator',
    #    default_value='True',
    #    description='Whether to start the simulator')

    config = os.path.join(
        get_package_share_directory('gps'),
        'params',
        'params.yaml')

    start_gps_transform=Node(
        #condition = IfCondition(use_simulator),
        package = 'gps',
        name = 'gps_transform',
        executable = 'gps_transform',
        parameters = [config])

    ld = LaunchDescription()
    ld.add_action(start_gps_transform)
    return ld
