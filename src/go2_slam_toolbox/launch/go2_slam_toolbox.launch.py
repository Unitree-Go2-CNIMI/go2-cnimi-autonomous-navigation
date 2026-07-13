from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    slam_toolbox_config = os.path.join(
        get_package_share_directory('go2_slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'slam_params_file': slam_toolbox_config,
            'use_sim_time': 'false',
        }.items(),
    )



    return LaunchDescription([
        slam_toolbox_launch

    ])