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
        default_value='/ros2_ws/src/razonamiento_package/config/slam_toolbox_params.yaml'
    )

    robot_name = LaunchConfiguration('robot_name')
    control_rate = LaunchConfiguration('control_rate')
    #auto_generate_goals = LaunchConfiguration('auto_generate_goals')

    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')

    slam_params_file = LaunchConfiguration('slam_params_file')

    # state_builder zones_file
    declare_zones_file = DeclareLaunchArgument(
        'zones_file',
        default_value="/ros2_ws/src/razonamiento_package/config/zones.yaml"
    )

    # experiment_file parameter
    declare_experiment_file = DeclareLaunchArgument(
        'experiment_file',
        default_value="/ros2_ws/src/razonamiento_package/config/single_goal.yaml",
        description='Path to experiment configuration YAML file'
    )


    coppelia_interface = Node(
        package='razonamiento_package',
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
        package='razonamiento_package',
        executable='bug2_controller_node',   # OJO: que este nombre exista en console_scripts
        name='bug2_controller',
        output='screen',
        parameters=[{
            'control_frequency': control_rate,       # antes control_rate
            'max_linear_speed': 4.0,
            'max_angular_speed': 15.0,
            'wheel_separation': 0.33,
            
            'm_line_tolerance': 0.3,
            'goal_reached_tolerance': 0.8,
            'obstacle_threshold': 0.4,
            'target_wall_distance': 0.5,
            'wall_distance_tolerance': 0.2,
            
            'angular_gain': 1.5,
            'forward_speed_ratio': 0.8,
            'wall_follow_speed': 1.4,
            
            'debug_log_frequency': 200,
            
            # === NUEVOS PARÁMETROS DE UNREACHABLE ===
            'enable_unreachable_detection': True,
            'max_distance_factor': 20.0,
            'max_state_changes': 50,
            'max_wall_follow_time': 120.0,
            'feedback_rate_hz': 1.0,
        }],
        emulate_tty=True
    )

    #goal_manager = Node(
    #    package='razonamiento_package',
    #    executable='goal_manager_node',
    #    name='goal_manager',
    #    output='screen',
    #    parameters=[{
    #        'goal_tolerance': 0.8,
    #        'min_goal_distance': 2.0,
    #        'map_min_x': -7.40,
    #        'map_max_x': 2.45,
    #        'map_min_y': -2.45,
    #        'map_max_y': 2.45,
    #        'map_buffer': 0.2,
    #        'auto_generate': auto_generate_goals,
    #        'goal_frame': 'world',
    #        'check_rate_hz': 10.0,
    #        'max_attempts': 200
    #    }],
    #    emulate_tty=True
    #)

    map_semantic_extractor = Node(
        package="razonamiento_package",   
        executable="map_semantic_extractor_node",    
        name="map_semantic_extractor",
        output="screen",
        parameters=[{
            'free_threshold': 99,        # Umbral para espacio libre (0-100)
            'occ_threshold': 50,         # Umbral para obstáculos (0-100)
            'min_clearance_m': 0.3,      # Clearance mínimo en metros
            'snap_radius_cells': 5,      # Radio para fusionar nodos cercanos
            'junction_degree': 3,        # Grado mínimo para considerar junction
            'morph_open_iters': 0,       # 0 = no erosionar (preserva puertas)
            'morph_close_iters': 1,      # 1 = cerrado suave (rellena huecos pequeños)
            'morph_kernel_size': 2,      # Tamaño del kernel (3x3 píxeles)
            'enable_frontiers': True,              # Activar/desactivar detección de frontiers
            'frontier_min_size_cells': 20,         # Tamaño mínimo de frontier (en celdas)
            'frontier_connectivity': 8,            # Conectividad para clustering (4 u 8)
            'frontier_use_8_connectivity': True,   # Usar 8-conectividad para dilatar unknown
            'zones_file': LaunchConfiguration('zones_file'),
            'default_zone_name': 'unknown',
        }],
        remappings=[
            ('/map', '/map'),  # Puedes cambiar el topic si es necesario
        ]
    )

    llm_state_builder = Node(
        package="razonamiento_package",  
        executable='llm_state_builder_node',
        name='llm_state_builder',
        output='screen',
        parameters=[{
            'compact_precision': 2,          # Decimales para floats
            'include_covariance': False,     # Incluir covarianza de pose
            'max_frontiers': 5,              # Máximo número de frontiers
            'max_graph_nodes': 20,           # Máximo número de nodos del grafo
            'sonar_threshold': 1.5,          # Distancia máxima relevante (m)
        }]
    )

    experiment_manager_node = Node(
        package='razonamiento_package',
        executable='experiment_manager_node',
        name='experiment_manager',
        output='screen',
        parameters=[{
            'experiment_file': LaunchConfiguration('experiment_file'),
            'goal_distance_threshold': 0.8,
            'check_rate_hz': 5.0,
        }],
    )

    llm_backend_node = Node(
        package='razonamiento_package',
        executable='llm_backend_node',
        name='llm_backend',
        output='screen',
        parameters=[{
            'enable_debug_logs': False
        }]
    )

    llm_orchestrator_node = Node(
        package='razonamiento_package',
        executable='llm_orchestrator_node',
        name='llm_orchestrator',
        output='screen',
        parameters=[{
            'mode': 'react',
            'map_min_x': -8.0,
            'map_min_y': -3.0,
            'map_max_x': 3.0,
            'map_max_y': 3.0
        }]
    )
    
    monitor_node = Node(
        package='razonamiento_package',
        executable='monitor_node',
        name='monitor',
        output='screen',
        parameters=[{
            'output_dir': './maps',
            'log_filename': 'prueba_1',  
            'buffer_size': 100
        }]
    )

    return LaunchDescription([
        declare_robot_name,
        declare_control_rate,
        #declare_auto_goals,
        declare_laser_x, declare_laser_y, declare_laser_z, declare_laser_yaw,
        declare_slam_params,
        declare_zones_file,
        declare_experiment_file,

        coppelia_interface,
        laser_static_tf,
        slam_toolbox,        # <- clave para /map y map->odom
        bug2_controller,
        #goal_manager,
        map_semantic_extractor,
        llm_state_builder,
        experiment_manager_node,
        llm_backend_node,
        llm_orchestrator_node,
        monitor_node


    ])
