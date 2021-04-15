# 
# Launch file to start gps sensor, capacitive touch and laser 1D
#

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gps_dir = get_package_share_directory('gps')
    

    # Create the launch configuration variables
    gps_port = LaunchConfiguration('gps_port')
    feather_port = LaunchConfiguration('feather_port')
    motor_port = LaunchConfiguration('motor_port')

    declare_gps_port_cmd = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyACM0',
        description='Port to communication with gps sensor')

    declare_feather_port_cmd = DeclareLaunchArgument(
        'feather_port',
        default_value='/dev/ttyACM1',
        description='Port to communicate with feather M0')

    declare_motor_port_cmd = DeclareLaunchArgument(
        'motor_port',
        default_value='/dev/ttyUSB0',
        description='Port to communicate with motors')

    # Create nodes for gps
    #start_serial0_cmd = Node(
    #    package="sgd_util",
    #    executable="serial",
    #    name="gps_serial",
    #    output="screen",
    #    emulate_tty=True,
    #    parameters=[
    #        {"port": gps_port,
    #         "baud_rate": 115200,
    #         "read_write": "ro"}     # one of ro,rw
    #    ])

    #start_gps_cmd = Node(
    #    package="gps",
    #    executable="navilock_ublox6_gps",
    #    name="gps_sensor",
    #    output="screen",
    #    emulate_tty=True,
    #    parameters=[
    #        {"port": gps_port,
    #         "xml_file": os.path.join(gps_dir,'data','nmea.xml')}
    #    ]
    #)

    # Create nodes for capacitive touch and laser 1D
    start_serial1_cmd = Node(
        package="sgd_util",
        executable="serial",
        name="sensor_serial",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": feather_port,
             "baud_rate": 115200,
             "read_write": "ro"}
        ])

    start_cap_touch_cmd = Node(
        package="cap_touch",
        executable="capacitive_touch",
        name="capacitive_touch",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": feather_port,
             "thresh": 2000,
             "filter_i": 4}
        ]
    )

    start_laser1d_cmd = Node(
        package="laser1d",
        executable="laser_1d",
        name="laser_1d",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": feather_port}
        ]
    )
    
    # Create nodes for motor control
    start_serial2_cmd = Node(
        package="sgd_util",
        executable="serial",
        name="motor_serial",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": motor_port,
             "baud_rate": 115200,
             "read_write": "rw"}
        ])
    
    start_motor_cmd = Node(
        package="sgd_lc",
        executable="local_controller",
        name="local_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": motor_port}
        ]
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_gps_port_cmd)
    ld.add_action(declare_feather_port_cmd)
    ld.add_action(declare_motor_port_cmd)

    # Add nodes
    #ld.add_action(start_serial0_cmd)
    #ld.add_action(start_gps_cmd)

    ld.add_action(start_serial1_cmd)
    ld.add_action(start_cap_touch_cmd)
    ld.add_action(start_laser1d_cmd)

    ld.add_action(start_serial2_cmd)
    ld.add_action(start_motor_cmd)

    return ld
