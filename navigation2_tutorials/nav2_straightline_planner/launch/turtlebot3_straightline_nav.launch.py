import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # TurtleBot3 model
    pkg_tb3_desc = FindPackageShare('turtlebot3_description').find('turtlebot3_description')
    urdf = os.path.join(pkg_tb3_desc, 'urdf', 'turtlebot3_waffle.urdf')
    
    # Bringup package
    pkg_nav2 = FindPackageShare('nav2_bringup').find('nav2_bringup')
    bringup_launch = os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
    
    # Params and map from your workspace
    pkg_ws = FindPackageShare('nav2_straightline_planner').find('nav2_straightline_planner')
    params_file = os.path.join(pkg_ws, 'params', 'nav2_params_straightline.yaml')
    map_file = os.path.join(pkg_ws, 'maps', 'map.yaml')
    
    # Arguments
    autostart = LaunchConfiguration('autostart', default='True')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='')
    
    declare_args = [
        DeclareLaunchArgument('autostart', default_value='True', description='Autostart Nav2'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Use sim time'),
        DeclareLaunchArgument('namespace', default_value='', description='Namespace')
    ]
    
    # 1) Gazebo server & client with empty world
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros').find('gazebo_ros'), '/launch/gzserver.launch.py']),
        launch_arguments={'world': os.path.join(FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo'),
                                                'worlds', 'empty.world')}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros').find('gazebo_ros'), '/launch/gzclient.launch.py'])
    )
    
    # 2) Spawn TurtleBot3 at non-obstacle pose (e.g., x=1,y=0)
    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-file', urdf,
            '-x', '1.0',   # spawna a x=1m
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # 3) Nav2 bringup (map_server, AMCL, planner, controller, BT)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'False',
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )
    
    # 4) RViz2 with default Nav2 view (most elements already config’d)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')]
    )
    
    return LaunchDescription(
        declare_args + [
            LogInfo(msg='Launching Gazebo with empty world…'),
            gzserver,
            gzclient,
            LogInfo(msg='Spawning TurtleBot3 at (1,0)…'),
            spawn_tb3,
            LogInfo(msg='Starting Nav2 bringup…'),
            nav2,
            LogInfo(msg='Starting RViz2…'),
            rviz,
        ]
    )
