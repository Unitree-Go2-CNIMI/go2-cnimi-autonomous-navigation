from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    description_pkg = FindPackageShare('go2_description')
    fast_lio_pkg = FindPackageShare('fast_lio')
    navigation_pkg = FindPackageShare('go2_navigation')
    livox_pkg = FindPackageShare('livox_ros_driver2')
    collision_monitor_pkg = FindPackageShare('nav2_collision_monitor')
    rosbridge_pkg = FindPackageShare('rosbridge_server')

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg.find('go2_description'),
                         'launch',
                         'description.launch.py')
        )
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_pkg.find('livox_ros_driver2'),
                         'launch_ROS2',
                         'msg_MID360_launch.py')
        )
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_pkg.find('rosbridge_server'),
                         'launch',
                         'rosbridge_websocket_launch.xml')
        )
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg.find('fast_lio'),
                         'launch',
                         'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'mid360.yaml'
        }.items()
    )

    livox_to_pc = Node(
        package='go2_tf_utils',
        executable='livox_to_ptcloud',
        output='screen'
    )

    odom_tf = Node(
        package='go2_tf_utils',
        executable='odom_base_tf',
        output='screen'
    )

    livox_tf = Node(
        package='go2_tf_utils',
        executable='livox_stabilized_tf',
        output='screen'
    )

    pc_to_scan = Node(
        package='go2_tf_utils',
        executable='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/livox/pointcloud2'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'livox_frame_stabilized',
            'transform_tolerance': 0.5,
            'min_height': 0.0,
            'max_height': 0.5,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.0087,
            'scan_time': 0.25,
            'range_min': 0.17,
            'range_max': 8.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg.find('go2_navigation'), 'launch', 'go2_amcl.launch.py')
        )
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg.find('go2_navigation'), 'launch', 'go2_nav.launch.py')
        )
    )

    mission_executor_action = Node(
        package='go2_mission',
        executable='mission_executor_action_executable',
        output='screen'
    )

    mission_api = Node(
        package='go2_mission',
        executable='mission_api_executable',
        output='screen'
    )

    collision_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(collision_monitor_pkg.find('nav2_collision_monitor'),
                         'launch',
                         'collision_monitor_node.launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(navigation_pkg.find('go2_navigation'), 'config', 'go2_nav2_params.yaml'),
            'use_sim_time': 'false',
        }.items()
    )

    fast_lio_init_delay          = 30.0
    tf_nodes_init_delay          = 40.0
    amcl_init_delay              = 50.0
    nav_init_delay               = 60.0
    collision_monitor_init_delay = 65.0
    execute_nav2_init_delay      = 70.0

    delayed_fast_lio = TimerAction(
        period=fast_lio_init_delay,
        actions=[fast_lio_launch]
    )

    delayed_nodes = TimerAction(
        period=tf_nodes_init_delay,
        actions=[
            odom_tf,
            livox_to_pc,
            livox_tf,
            pc_to_scan,
        ]
    )

    delayed_amcl = TimerAction(
        period=amcl_init_delay,
        actions=[amcl_launch]
    )

    delayed_nav = TimerAction(
        period=nav_init_delay,
        actions=[
            nav_launch,
            mission_executor_action,
            mission_api,
        ]
    )

    delayed_collision_monitor = TimerAction(
        period=collision_monitor_init_delay,
        actions=[
            collision_monitor_launch,
        ]
    )

    execute_nav2 = Node(
        package='unitree_ros2_example',
        executable='execute_nav2',
        output='screen'
    )

    delayed_execute_nav2 = TimerAction(
        period=execute_nav2_init_delay,
        actions=[execute_nav2]
    )

    return LaunchDescription([
        description_launch,
        livox_launch,
        rosbridge_launch,
        delayed_fast_lio,
        delayed_nodes,
        delayed_amcl,
        delayed_nav,
        delayed_collision_monitor,
        delayed_execute_nav2,
    ])
