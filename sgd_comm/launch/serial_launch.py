import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    lifecycle_nodes = ['gps_serial']

    return LaunchDescription([
        Node(
            package="sgd_comm",
            executable="serial",
            name="gps_serial",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": "/dev/ttyACM0",
                 "baud_rate": 9600,
                 "read_write": "ro",
                 "logfile": os.path.join('/home/ipp/dev_ws','log','serial_gps.log'),
                 "raw": True,
                 "sframe": '\n',
                 "stframe": '\n',
                 "log": True}]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_hardware',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
    ])
