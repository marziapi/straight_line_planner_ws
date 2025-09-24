import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate packages
    pkg_gazebo = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_tb3_gazebo = FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo')
    pkg_tb3_desc = FindPackageShare('turtlebot3_description').find('turtlebot3_description')
    pkg_nav2 = FindPackageShare('nav2_bringup').find('nav2_bringup')
    pkg_ws = FindPackageShare('nav2_straightline_planner').find('nav2_straightline_planner')

    # Paths
    world_file = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')
    urdf_file = os.path.join(pkg_tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    map_yaml  = os.path.join(pkg_ws, 'maps', 'map.yaml')
    params    = os.path.join(pkg_ws, 'params', 'nav2_params_straightline.yaml')
    rviz_cfg  = os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart    = LaunchConfiguration('autostart',    default='True')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='True',
                               description='Use simulation clock'),
        DeclareLaunchArgument('autostart',    default_value='True',
                               description='Automatically start Nav2')
    ]

    # Gazebo server & client
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py'))
    )

    # Spawn TurtleBot3 at origin
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_waffle',
            '-file', urdf_file,
            '-x', '-2.0', '-y', '-1.0', '-z', '0.01', '-Y', '0.0'
        ],
        output='screen'
    )

    # Nav2 bringup (map_server, amcl localization, planner, controller, BT)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params,
            'autostart': autostart
        }.items()
    )

    # After 3s, configure the lifecycle manager
    configure_lm = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='Configuring lifecycle_manager_localization...'),
            ExecuteProcess(cmd=[
                'ros2', 'lifecycle', 'set',
                '/lifecycle_manager_localization', 'configure'
            ])
        ]
    )

    # After 4s, activate the lifecycle manager
    activate_lm = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg='Activating lifecycle_manager_localization...'),
            ExecuteProcess(cmd=[
                'ros2', 'lifecycle', 'set',
                '/lifecycle_manager_localization', 'activate'
            ])
        ]
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        *declare_args,
        LogInfo(msg='Launching Gazebo world…'),
        gzserver,
        gzclient,
        LogInfo(msg='Spawning TurtleBot3…'),
        spawn_tb3,
        LogInfo(msg='Starting Nav2 bringup…'),
        nav2,
        configure_lm,
        activate_lm,
        LogInfo(msg='Launching RViz…'),
        rviz,
    ])
