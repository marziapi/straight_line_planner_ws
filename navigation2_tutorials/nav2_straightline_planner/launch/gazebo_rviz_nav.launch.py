import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Parametri di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart    = LaunchConfiguration('autostart', default='True')
    headless = LaunchConfiguration('headless', default='False')

    # Percorsi ai package
    pkg_tb3_sim = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    pkg_nav2    = FindPackageShare('nav2_bringup').find('nav2_bringup')
    pkg_ws      = FindPackageShare('nav2_straightline_planner').find('nav2_straightline_planner')

    # File di mappa, parametri e RViz
    map_yaml = os.path.join(pkg_nav2, 'maps', 'map.yaml')
    params   = os.path.join(pkg_ws, 'params', 'nav2_params_straightline.yaml')
    rviz_cfg = os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')

    # Argomenti dichiarati
    declare_args = [
        #DeclareLaunchArgument('use_sim_time', default_value='True'),
        #DeclareLaunchArgument('autostart',    default_value='True')
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('params_file', default_value=params,
                            description='Full path to nav2 params file')
    ]

    # 1) Launch di TurtleBot3 in Gazebo
    tb3_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            #'use_sim_time': use_sim_time,
            #'headless' : headless,
            'use_sim_time': use_sim_time,
            'headless': headless,
            'params_file': params,
        }.items()
    )

    return LaunchDescription([
        *declare_args,
        tb3_sim_launch,
    ])