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
    sgd_util_dir = get_package_share_directory('sgd_util')
    
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    
    gps_port = LaunchConfiguration('gps_port')
    feather_port = LaunchConfiguration('feather_port')
    esp_port = LaunchConfiguration('esp_port')

    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_gps_port_cmd = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyACM0',
        description='Port to communication with gps sensor')

    declare_feather_port_cmd = DeclareLaunchArgument(
        'feather_port',
        default_value='/dev/ttyACM0',
        description='Port to communicate with feather M0')

    declare_motor_port_cmd = DeclareLaunchArgument(
        'esp_port',
        default_value='/dev/ttyUSB0',
        description='Port to communicate with motors')

    lifecycle_nodes = ['gps_serial', 'gps_sensor']

    # Create nodes for gps
    start_serial0_cmd = Node(
        package="sgd_util",
        executable="serial_mock",
        name="gps_serial",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": gps_port,
             "logfile": os.path.join(sgd_util_dir,'log','serial.log'),
             "baud_rate": 115200,
             "read_write": "ro",
             "raw": True,
             "sframe": '\n',
             "log": True}])

    start_gps_cmd = Node(
        package="gps",
        executable="navilock_ublox6_gps",
        name="gps_sensor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": gps_port,
             "xml_file": os.path.join(gps_dir,'data','nmea.xml')}])

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
        ]
    )

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
        name="esp_serial",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": esp_port,
             "baud_rate": 115200,
             "read_write": "rw"}
        ]
    )
    
    start_motor_driver_cmd = Node(
        package="sgd_lc",
        executable="wheel_driver",
        name="wheel_driver",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": esp_port,
             "motor_kp": 0.3,
             "max_speed": 200}
        ]
    )

    start_motor_cmd = Node(
        package="sgd_lc",
        executable="local_controller",
        name="local_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"filter_i": 5,
             "speed_kp": 0.4,
             "turn_kp": 0.4}
        ]
    )

    start_imu_cmd = Node(
        package="imu",
        executable="imu_bno055",
        name="imu_bno055",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"port": esp_port,
             "config_mode": True}
        ]
    )

    lifecycle_manager_sensors = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sensors',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_sim_time_cmd)
    ld.add_action(declare_gps_port_cmd)
    ld.add_action(declare_feather_port_cmd)
    ld.add_action(declare_motor_port_cmd)

    # Add nodes
    ld.add_action(start_serial0_cmd)
    ld.add_action(start_gps_cmd)

    #ld.add_action(start_serial1_cmd)
    #ld.add_action(start_cap_touch_cmd)
    #ld.add_action(start_laser1d_cmd)

    #ld.add_action(start_serial2_cmd)
    #ld.add_action(start_motor_driver_cmd)
    #ld.add_action(start_motor_cmd)
    #ld.add_action(start_imu_cmd)

    ld.add_action(lifecycle_manager_sensors)

    return ld
