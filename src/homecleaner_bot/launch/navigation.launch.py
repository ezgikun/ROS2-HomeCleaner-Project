import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'homecleaner_bot'
    pkg_share = get_package_share_directory(pkg_name)

    map_file_path = os.path.join(pkg_share, 'maps', 'my_home_map.yaml')
    
    nav2_params_path = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')

    simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')
        )
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,
            'use_sim_time': 'true',
            'params_file': nav2_params_path,
            'autostart': 'true'  # Otomatik baslat
        }.items()
    )

    return LaunchDescription([
        simulation_cmd,
        nav2_cmd
    ])
