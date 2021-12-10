from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
 
        Node(
            package="sgd_controller",
            executable="lidar_obstacle_checker",
            name="lidar_obstacle_checker",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"scan_topic": "scan",
                 "cmd_vel_contr_topic": "cmd_vel",
                 "sgd_move_topic": "cmd_vel_lidar"}]),

        Node(
            package='sgd_test',
            executable='lifecycle_manager_test',
            name='lifecycle_mgr_test',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"launch_node": "lidar_obstacle_checker"}]),

        ros_playback_cmd = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-o', 'testdata'],
            output='screen')
    ])
