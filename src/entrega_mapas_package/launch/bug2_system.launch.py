#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    declare_robot_name = DeclareLaunchArgument('robot_name', default_value='Pioneer_p3dx')
    declare_control_rate = DeclareLaunchArgument('control_rate', default_value='20.0')
    declare_auto_goals = DeclareLaunchArgument('auto_generate_goals', default_value='true')

    # TF laser respecto a base_link
    declare_laser_x = DeclareLaunchArgument('laser_x', default_value='0.20')
    declare_laser_y = DeclareLaunchArgument('laser_y', default_value='0.00')
    declare_laser_z = DeclareLaunchArgument('laser_z', default_value='0.20')
    declare_laser_yaw = DeclareLaunchArgument('laser_yaw', default_value='0.0')

    # slam_toolbox params
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value='/ros2_ws/src/entrega_mapas_package/config/slam_toolbox_params.yaml'
    )

    robot_name = LaunchConfiguration('robot_name')
    control_rate = LaunchConfiguration('control_rate')
    auto_generate_goals = LaunchConfiguration('auto_generate_goals')

    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    slam_params_file = LaunchConfiguration('slam_params_file')

    coppelia_interface = Node(
        package='entrega_mapas_package',
        executable='coppelia_interface_node',
        name='coppelia_interface',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'update_rate': control_rate,
            'max_speed': 2.0,
            'scan_frame': 'laser'
        }],
        emulate_tty=True
    )

    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        output='screen',
        arguments=[
            laser_x, laser_y, laser_z,
            '0.0', '0.0', laser_yaw,
            'base_link', 'laser'
        ]
    )

    # SLAM: publica /map y TF map->odom
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    bug2_controller = Node(
        package='entrega_mapas_package',
        executable='bug2_controller_node',   # OJO: que este nombre exista en console_scripts
        name='bug2_controller',
        output='screen',
        parameters=[{
            'control_frequency': control_rate,       # antes control_rate
            'max_linear_speed': 4.0,
            'max_angular_speed': 15.0,
            'wheel_separation': 0.33,


            'goal_reached_tolerance': 0.8,           # antes goal_tolerance
            'm_line_tolerance': 0.2,

            'obstacle_threshold': 0.6,               # ajustable
            'target_wall_distance': 0.75,            # antes wall_distance
            'wall_distance_tolerance': 0.5,

            'angular_gain': 2.0,
            'forward_speed_ratio': 0.7,
            'wall_follow_speed': 1.0,                # MUY importante (ver punto 2)
            'debug_log_frequency': 200
        }],
        emulate_tty=True
    )




    goal_manager = Node(
        package='entrega_mapas_package',
        executable='goal_manager_node',
        name='goal_manager',
        output='screen',
        parameters=[{
            'goal_tolerance': 0.8,
            'min_goal_distance': 2.0,
            'map_min_x': -7.40,
            'map_max_x': 2.45,
            'map_min_y': -2.45,
            'map_max_y': 2.45,
            'map_buffer': 0.2,
            'auto_generate': auto_generate_goals,
            'goal_frame': 'world',
            'check_rate_hz': 10.0,
            'max_attempts': 200
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        declare_robot_name,
        declare_control_rate,
        declare_auto_goals,
        declare_laser_x, declare_laser_y, declare_laser_z, declare_laser_yaw,
        declare_slam_params,

        coppelia_interface,
        laser_static_tf,
        slam_toolbox,        # <- clave para /map y map->odom
        bug2_controller,
        goal_manager,
    ])
