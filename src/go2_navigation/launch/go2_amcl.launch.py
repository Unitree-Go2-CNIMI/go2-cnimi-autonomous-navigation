import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    map_name     = LaunchConfiguration("map_name")
    map_yaml     = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config  = LaunchConfiguration("amcl_config")
    lifecycle_nodes = ["map_server", "amcl"]

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="map_fast_edited",
        description="Sub-directory under go2_navigation/maps/ containing the map files"
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="map_fast_edited.yaml",
        description="YAML filename inside the map directory (e.g. my_map.yaml, map_fast3_corrected.yaml)"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("go2_navigation"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    map_path = PathJoinSubstitution([
        get_package_share_directory("go2_navigation"),
        "maps",
        map_name,
        map_yaml,
    ])
    
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        map_name_arg,
        map_yaml_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
        rviz_launch
    ])