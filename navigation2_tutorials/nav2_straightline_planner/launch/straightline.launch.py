# Author: Modified for TurtleBot3 with straight-line planner - LOCALIZATION MODE
# Date: September 24, 2025

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # TurtleBot3 Configuration
    package_name = 'turtlebot3_description'
    robot_name_in_model = 'turtlebot3_waffle'
    
    # TurtleBot3 specific paths
    urdf_file_path = 'urdf/turtlebot3_waffle.urdf'
    world_file_path = 'turtlebot3_world.world'
    
    # Your custom parameters file path
    nav2_params_path = 'params/nav2_params_straightline.yaml'

    # TurtleBot3 spawn position
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.01'
    spawn_yaw_val = '0.0'

    # Set the path to different files and folders
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_turtlebot3_gazebo = FindPackageShare(package='turtlebot3_gazebo').find('turtlebot3_gazebo')
    pkg_turtlebot3_description = FindPackageShare(package=package_name).find(package_name)
    
    # Try to find TurtleBot3 navigation2 package for default map
    try:
        pkg_turtlebot3_navigation2 = FindPackageShare(package='turtlebot3_navigation2').find('turtlebot3_navigation2')
        default_map_path = os.path.join(pkg_turtlebot3_navigation2, 'maps', 'map.yaml')
    except:
        # Fallback: use your own map or create a dummy one
        your_workspace_pkg = 'nav2_straightline_planner'
        pkg_your_workspace = FindPackageShare(package=your_workspace_pkg).find(your_workspace_pkg)
        default_map_path = os.path.join(pkg_your_workspace, 'maps', 'empty_map.yaml')
    
    # Your workspace package
    your_workspace_pkg = 'nav2_straightline_planner'
    pkg_your_workspace = FindPackageShare(package=your_workspace_pkg).find(your_workspace_pkg)
    
    default_urdf_model_path = os.path.join(pkg_turtlebot3_description, urdf_file_path)
    nav2_params_path = os.path.join(pkg_your_workspace, nav2_params_path)
    world_path = os.path.join(pkg_turtlebot3_gazebo, 'worlds', world_file_path)
    
    # Nav2 directories
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    
    # Launch configuration variables - DICHIARATE TUTTE QUI
    autostart = LaunchConfiguration('autostart')
    headless = LaunchConfiguration('headless')
    map_yaml_file = LaunchConfiguration('map')  # QUESTA ERA MANCANTE PRIMA
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Declare ALL launch arguments BEFORE using them
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
            
    declare_autostart_cmd = DeclareLaunchArgument(
        name='autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map',
        default_value=default_map_path,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')
        
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    
    # Actions
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    spawn_turtlebot3_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        arguments=['-entity', robot_name_in_model,
                   '-file', default_urdf_model_path,
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                   '-Y', spawn_yaw_val],
        output='screen')

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time,
        'robot_description': Command(['xacro ', urdf_model])}],
        remappings=remappings)
    
    start_turtlebot3_drive_cmd = Node(
        package='turtlebot3_gazebo',
        executable='turtlebot3_drive',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen')

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen')

    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,  # ORA È DICHIARATO CORRETTAMENTE
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items())

    # Create the launch description
    ld = LaunchDescription()

    # IMPORTANTE: Aggiungere TUTTE le dichiarazioni PRIMA delle azioni
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd) 
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_turtlebot3_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_turtlebot3_drive_cmd) 
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_ros2_navigation_cmd)

    return ld