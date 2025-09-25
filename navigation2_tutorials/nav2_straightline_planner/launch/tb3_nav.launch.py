import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Percorsi ai package
    pkg_nav2 = FindPackageShare('nav2_bringup').find('nav2_bringup')
    pkg_ws  = FindPackageShare('nav2_straightline_planner').find('nav2_straightline_planner')

    # File di mappa e parametri
    default_map   = os.path.join(pkg_nav2, 'maps', 'turtlebot3_world.yaml')
    default_params= os.path.join(pkg_ws,   'params', 'nav2_params_straightline.yaml')

    # Argomenti di launch
    map_arg    = DeclareLaunchArgument('map',        default_value=default_map,
                                       description='Path to map file')
    params_arg = DeclareLaunchArgument('params_file',default_value=default_params,
                                       description='Path to Nav2 parameters YAML')
    use_sim    = DeclareLaunchArgument('use_sim_time',default_value='True')
    headless   = DeclareLaunchArgument('headless',    default_value='False')
    autostart  = DeclareLaunchArgument('autostart',   default_value='True')

    # Include del launch originale con map esplicito
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'map':        LaunchConfiguration('map'),
            'params_file':LaunchConfiguration('params_file'),
            'use_sim_time':LaunchConfiguration('use_sim_time'),
            'headless':   LaunchConfiguration('headless'),
            'autostart':  LaunchConfiguration('autostart'),
        }.items()
    )

    return LaunchDescription([
        map_arg, params_arg, use_sim, headless, autostart,
        bringup
    ])