from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sgd_lc",
            executable="wheel_driver",
            name="wheel_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": "/dev/ttyUSB0",
                 "motor_kp": 0.3,
                 "max_speed": 200}
            ]
        )
    ])
