from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sgd_util",
            executable="serial",
            name="custom_serial",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": "/dev/ttyACM0",
                 "baud_rate": 9600,
                 "read_write": "ro"}     # one of ro,rw}
            ]
        )
    ])
