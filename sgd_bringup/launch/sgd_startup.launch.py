"""This is all-in-one launch script to run the shared guide dog."""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, EmitEvent, RegisterEventHandler, SetEnvironmentVariable, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('sgd_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Launch variables
    sim = LaunchConfiguration('sim')
    slam = LaunchConfiguration('slam')

    # Create the launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    # Variables for logging
    log_dir = LaunchConfiguration('log_dir')
    log_severity = LaunchConfiguration('log_severity')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_sim_cmd = DeclareLaunchArgument(
        'sim',
        default_value='False',
        description='Whether to start the simulator')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'lohmuehlenpark.yaml'),
        #default_value=os.path.join(bringup_dir, 'maps', 'augustinum.yaml'),
        description='Full path to map file to load')
        
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'sgd_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_hardware_params_file_cmd = DeclareLaunchArgument(
        'hardware_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'hardware_params.yaml'),
        description='Full path to hardware specific parameters.')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            bringup_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery_sgd.xml'),
        description='Full path to the behavior tree xml file to use')

    # create log dir
    now = datetime.now()
    dtime = now.strftime("%Y-%m-%d-%H-%M-%S")
    # create directory because plog cannot create a directory
    path = os.path.join(os.path.expanduser('~'), '.ros', 'log', dtime)
    plog_path = os.path.join(path, 'data')
    os.makedirs(plog_path)
    set_env_log = SetEnvironmentVariable('ROS_LOG_DIR', path)
    declare_log_dir_cmd = DeclareLaunchArgument(
        'log_dir',
        default_value=plog_path,
        description='Path to log directory'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'sgd_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'lohmuehlenpark.model'),
        description='Full path to world model file to load')

    param_substitutions = {
        'use_sim_time': sim,
        'log_dir': log_dir,
        'default_bt_xml_filename': default_bt_xml_filename,
        'map_subscribe_transient_local': 'true',
        'yaml_filename': map_yaml_file,
        'tag_defs_file': os.path.join(bringup_dir, 'config', 'uwb_tag_defs.yaml'),
        'parser_file': os.path.join(bringup_dir, 'config', 'nmea.xml'),
        'voice_msgs_dir': os.path.join(bringup_dir, 'voice_msgs')}

    configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True)

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(sim),
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', world],
        cwd=[launch_dir], output={'both': 'log'})

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([sim, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    urdf = os.path.join(bringup_dir, 'urdf', 'sgd_model.urdf')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': sim}],
        remappings=remappings,
        arguments=[urdf])

    # Launch rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output={'both': 'log'})

    # exit_event_handler = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=None,
    #         on_exit=LogInfo(msg='process exited')))

    ## on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))

    ## start nodes
    slam_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'use_sim_time': sim,
                              'params_file': configured_params}.items())

    localization_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'map': map_yaml_file,
                              'use_sim_time': sim,
                              'params': configured_params}.items())

    navigation_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'use_sim_time': sim,
                              'params': configured_params}.items())

    interaction_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'interaction_launch.py')),
            launch_arguments={'use_sim_time': sim,
                              'params': configured_params}.items())

    hardware_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'hardware_launch.py')),
        #condition=IfCondition(PythonExpression(['not ', sim])),
        launch_arguments={'map': map_yaml_file,
                          #'use_sim_time': sim,
                          'params': configured_params}.items())
    
    start_lifecycle_cmd = Node(
        package='sgd_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{"launch_file": os.path.join(bringup_dir, 'config', 'launch.xml')},
                    {"use_sim_time": sim}])

    start_mcu_cmd = Node(
        package='sgd_controller',
        executable='master_control_unit',
        name='master_control_unit',
        output='screen',
        emulate_tty=True)

    # start websocket
    start_rosbridge_websocket_cmd = ExecuteProcess(
        #condition=IfCondition(sim),
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        cwd=[launch_dir], output={'both': 'log'})

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_sim_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_hardware_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_log_dir_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(set_env_log)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(interaction_cmd)
    ld.add_action(hardware_cmd)

    ld.add_action(start_lifecycle_cmd)
    ld.add_action(start_mcu_cmd)

    ld.add_action(start_rosbridge_websocket_cmd)

    ld.add_action(start_rviz_cmd)
    ## add event handler
    # ld.add_action(exit_event_handler)

    return ld