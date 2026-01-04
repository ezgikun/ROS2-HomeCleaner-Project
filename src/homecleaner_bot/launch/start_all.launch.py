import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'homecleaner_bot'
    pkg_dir = get_package_share_directory(pkg_name)
    nav2_dir = get_package_share_directory('nav2_bringup')

    map_file_path = '/root/homecleaner_ws/src/homecleaner_bot/maps/my_home_map.yaml'

    rviz_config_path = '/root/homecleaner_ws/src/homecleaner_bot/my_nav.rviz'

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'simulation.launch.py')
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': 'true',
            'params_file': os.path.join(nav2_dir, 'params', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )

    rviz = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rviz2', 'rviz2','-d', rviz_config_path],
                output='screen'
            )
        ]
    )

    cleaner_script = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '/root/homecleaner_ws/src/homecleaner_bot/cleaner_manager.py'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        simulation,
        navigation,
        rviz,
        cleaner_script
    ])
