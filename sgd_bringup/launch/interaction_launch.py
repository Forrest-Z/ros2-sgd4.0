
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params')

    declare_sim_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    start_audio_cmd = Node(
        package='sgd_audio',
        executable='sgd_audio',
        name='sgd_audio',
        output='screen',
        emulate_tty=True,
        parameters=[params])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_sim_cmd)

    # nodes publishing transforms
    ld.add_action(start_audio_cmd)

    return ld