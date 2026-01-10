#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from std_msgs.msg import String
import math


class Bug2ControllerNode(Node):
    def __init__(self):
        super().__init__('bug2_controller_node')
        
        # ============= PARÁMETROS CONFIGURABLES (ORIGINALES) =============
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('wheel_separation', 0.33)
        self.declare_parameter('control_frequency', 20.0)
        
        self.declare_parameter('m_line_tolerance', 0.3)
        self.declare_parameter('goal_reached_tolerance', 0.8)
        self.declare_parameter('obstacle_threshold', 0.4)
        self.declare_parameter('target_wall_distance', 0.5)
        self.declare_parameter('wall_distance_tolerance', 0.2)
        
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('forward_speed_ratio', 0.8)
        self.declare_parameter('wall_follow_speed', 1.4)
        
        self.declare_parameter('debug_log_frequency', 200)
        
        # ============= NUEVOS PARÁMETROS DE UNREACHABLE =============
        self.declare_parameter('enable_unreachable_detection', True)
        self.declare_parameter('max_distance_factor', 5.0)
        self.declare_parameter('max_state_changes', 30)
        self.declare_parameter('max_wall_follow_time', 45.0)
        self.declare_parameter('feedback_rate_hz', 2.0)
        
        # Obtener parámetros originales
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        self.m_line_tolerance = self.get_parameter('m_line_tolerance').value
        self.goal_reached_tolerance = self.get_parameter('goal_reached_tolerance').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.target_wall_distance = self.get_parameter('target_wall_distance').value
        self.wall_distance_tolerance = self.get_parameter('wall_distance_tolerance').value
        
        self.angular_gain = self.get_parameter('angular_gain').value
        self.forward_speed_ratio = self.get_parameter('forward_speed_ratio').value
        self.wall_follow_speed = self.get_parameter('wall_follow_speed').value
        
        self.debug_log_frequency = self.get_parameter('debug_log_frequency').value
        
        # ============= ESTADOS BUG2 (ORIGINALES) =============
        self.MOTION_TO_GOAL = 0
        self.WALL_FOLLOWING = 1
        self.REACHED = 2
        self.current_state = self.MOTION_TO_GOAL
        
        # ============= VARIABLES BUG2 (ORIGINALES) =============
        self.start_point = None
        self.goal_point = None
        self.hit_point = None
        self.hit_distance_to_goal = None
        self.wall_following_side = 'right'
        
        self.current_pose = None
        self.current_position = [0.0, 0.0]
        self.current_orientation = 0.0
        
        self.sonar_readings = {}
        self.num_sonars = 16
        
        self.sensor_angles = {
            1: -90, 2: -50, 3: -30, 4: -10, 5: 10, 6: 30, 7: 50, 8: 90,
            9: 90, 10: 130, 11: 150, 12: 170, 13: -170, 14: -150, 15: -130, 16: -90
        }
        
        self.iteration = 0
        
        # ============= NUEVAS VARIABLES DE TRACKING =============
        self.initial_distance_to_goal = None
        self.total_distance_traveled = 0.0
        self.last_position = None
        self.state_changes = 0
        self.last_bug2_state = self.MOTION_TO_GOAL
        self.wall_follow_start_time = None
        self.goal_start_time = None
        self.unreachable_reported = False  # Flag para reportar solo una vez
        
        # ============= QoS =============
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,#TRANSIENT_LOCAL,
            depth=1,
            history=HistoryPolicy.KEEP_LAST 
        )
        feedback_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # ============= PUBLISHERS =============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, '/bug2/feedback', feedback_qos)
        
        # ============= SUBSCRIBERS =============
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot/pose', self.pose_callback, 10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal', self.goal_callback, goal_qos
        )
        
        # Suscribirse a todos los sonares
        self.sonar_subs = []
        for i in range(1, self.num_sonars + 1):
            sub = self.create_subscription(
                Range, f'/robot/sonar_{i}',
                lambda msg, sensor_id=i: self.sonar_callback(msg, sensor_id),
                10
            )
            self.sonar_subs.append(sub)
        
        # ============= TIMERS =============
        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.feedback_timer = self.create_timer(
            1.0 / self.get_parameter('feedback_rate_hz').value,
            self.publish_periodic_feedback
        )
        
        self.get_logger().info('=== Bug2 Controller Node Iniciado (Enhanced) ===')
        self.get_logger().info(f'Frecuencia de control: {self.control_frequency} Hz')
        self.get_logger().info(f'Velocidad máxima lineal: {self.max_linear_speed} m/s')
        self.get_logger().info(f'Velocidad máxima angular: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'M-line tolerance: {self.m_line_tolerance} m')
        self.get_logger().info(f'Goal tolerance: {self.goal_reached_tolerance} m')
        self.get_logger().info(f'Unreachable detection: {self.get_parameter("enable_unreachable_detection").value}')
        self.get_logger().info('Esperando pose, goal y sonares...')
    
    # ============= CALLBACKS =============
    
    def pose_callback(self, msg):
        """Callback de pose del robot"""
        # Actualizar distancia recorrida para tracking de unreachable
        if self.last_position is not None and self.current_state != self.REACHED:
            dx = msg.pose.position.x - self.last_position[0]
            dy = msg.pose.position.y - self.last_position[1]
            distance_increment = math.sqrt(dx*dx + dy*dy)
            self.total_distance_traveled += distance_increment
        
        self.current_pose = msg
        self.current_position = [msg.pose.position.x, msg.pose.position.y]
        self.last_position = self.current_position.copy()
        
        # Extraer yaw de quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def goal_callback(self, msg):
        """Callback de meta"""
        new_goal = [msg.pose.position.x, msg.pose.position.y]
        
        # Si es una meta nueva, resetear Bug2
        if self.goal_point is None or new_goal != self.goal_point:
            self.goal_point = new_goal
            
            if self.current_position is not None:
                self.start_point = self.current_position.copy()
            
            # Resetear estado Bug2
            self.current_state = self.MOTION_TO_GOAL
            self.last_bug2_state = self.MOTION_TO_GOAL
            self.hit_point = None
            self.hit_distance_to_goal = None
            
            # Resetear tracking de unreachable
            self.initial_distance_to_goal = self.distance_to_goal(self.current_position)
            self.total_distance_traveled = 0.0
            self.state_changes = 0
            self.wall_follow_start_time = None
            self.goal_start_time = self.get_clock().now()
            self.unreachable_reported = False  # Resetear flag
            
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'NUEVA META RECIBIDA: ({new_goal[0]:.2f}, {new_goal[1]:.2f})')
            self.get_logger().info(f'Start point (M-line): ({self.start_point[0]:.2f}, {self.start_point[1]:.2f})')
            self.get_logger().info(f'Distancia inicial: {self.initial_distance_to_goal:.2f} m')
            self.get_logger().info(f'Estado: MOTION_TO_GOAL')
            self.get_logger().info('=' * 50)
            
            self.publish_feedback("STARTED")
    
    def sonar_callback(self, msg, sensor_id):
        """Callback de sensores sonar"""
        self.sonar_readings[sensor_id] = msg.range
    
    # ============= FUNCIONES AUXILIARES (ORIGINALES) =============
    
    def distance_to_goal(self, position):
        """Calcula distancia euclidiana a la meta"""
        if self.goal_point is None:
            return float('inf')
        
        dx = self.goal_point[0] - position[0]
        dy = self.goal_point[1] - position[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def angle_to_goal(self, position, orientation):
        """Calcula ángulo hacia la meta"""
        if self.goal_point is None:
            return 0.0
        
        dx = self.goal_point[0] - position[0]
        dy = self.goal_point[1] - position[1]
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - orientation
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def is_on_m_line(self, position):
        """Verifica si la posición está en la M-line"""
        if self.start_point is None or self.goal_point is None:
            return False
        
        x1, y1 = self.start_point
        x2, y2 = self.goal_point
        x0, y0 = position
        
        line_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        if line_length < 0.1:
            return True
        
        perpendicular_distance = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / line_length
        
        return perpendicular_distance <= self.m_line_tolerance
    
    def distance_along_m_line_to_goal(self, position):
        """Calcula distancia a lo largo de M-line hacia la meta"""
        if self.start_point is None or self.goal_point is None:
            return float('inf')
        
        x1, y1 = self.start_point
        x2, y2 = self.goal_point
        x0, y0 = position
        
        line_vec = [x2 - x1, y2 - y1]
        point_vec = [x0 - x1, y0 - y1]
        
        line_length = math.sqrt(line_vec[0]**2 + line_vec[1]**2)
        
        if line_length < 0.1:
            return self.distance_to_goal(position)
        
        projection = (point_vec[0] * line_vec[0] + point_vec[1] * line_vec[1]) / (line_length**2)
        
        proj_x = x1 + projection * line_vec[0]
        proj_y = y1 + projection * line_vec[1]
        
        return math.sqrt((x2 - proj_x)**2 + (y2 - proj_y)**2)
    
    def get_sonar_readings_dict(self):
        """Convierte lecturas de sonares a formato similar a minimal.py"""
        readings = {}
        for sensor_id in range(1, self.num_sonars + 1):
            distance = self.sonar_readings.get(sensor_id, 1.5)
            readings[f'sonar_{sensor_id}'] = {
                'distance': distance,
                'angle': self.sensor_angles.get(sensor_id, 0),
                'valid': distance < 0.5,
                'sensor_id': sensor_id
            }
        return readings
    
    def is_path_to_goal_clear(self, sensor_readings, angle_to_goal):
        """Verifica si el camino directo a la meta está libre"""
        front_sensors = ['sonar_4', 'sonar_5']
        
        for sensor_name in front_sensors:
            if sensor_name in sensor_readings:
                reading = sensor_readings[sensor_name]
                if reading['valid'] and reading['distance'] < self.obstacle_threshold:
                    return False
        
        for sensor_name, reading in sensor_readings.items():
            if not reading['valid']:
                continue
            
            sensor_angle = math.radians(reading['angle'])
            angle_diff = abs(sensor_angle - angle_to_goal)
            
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            if angle_diff < math.pi/4:
                if reading['distance'] < self.obstacle_threshold:
                    return False
        
        return True
    
    def get_clearance_in_direction(self, sensor_readings, direction):
        """Calcula espacio libre en una dirección"""
        if direction == 'left':
            relevant_sensors = ['sonar_1', 'sonar_2', 'sonar_3', 'sonar_4']
        else:
            relevant_sensors = ['sonar_5', 'sonar_6', 'sonar_7', 'sonar_8']
        
        distances = []
        for sensor_name in relevant_sensors:
            if sensor_name in sensor_readings and sensor_readings[sensor_name]['valid']:
                distances.append(sensor_readings[sensor_name]['distance'])
        
        return sum(distances) / len(distances) if distances else 0.0
    
    # ============= NUEVA: DETECCIÓN DE UNREACHABLE =============
    
    def check_unreachable(self) -> tuple:
        """
        Verifica si el objetivo es inalcanzable según múltiples criterios.
        Retorna (is_unreachable: bool, reason: str)
        NO detiene el robot, solo reporta.
        """
        if not self.get_parameter('enable_unreachable_detection').value:
            return (False, "")
        
        # Criterio 1: Distancia recorrida excesiva
        if self.initial_distance_to_goal is not None and self.initial_distance_to_goal > 0:
            distance_factor = self.total_distance_traveled / self.initial_distance_to_goal
            max_factor = self.get_parameter('max_distance_factor').value
            
            if distance_factor > max_factor:
                reason = (
                    f"UNREACHABLE: Distance factor {distance_factor:.2f} > {max_factor:.2f} "
                    f"(traveled {self.total_distance_traveled:.1f}m, initial {self.initial_distance_to_goal:.1f}m)"
                )
                return (True, reason)
        
        # Criterio 2: Demasiados cambios de estado
        max_changes = self.get_parameter('max_state_changes').value
        if self.state_changes > max_changes:
            reason = f"UNREACHABLE: Too many state changes ({self.state_changes} > {max_changes})"
            return (True, reason)
        
        # Criterio 3: Siguiendo pared por demasiado tiempo
        if self.current_state == self.WALL_FOLLOWING and self.wall_follow_start_time is not None:
            now = self.get_clock().now()
            wall_follow_duration = (now - self.wall_follow_start_time).nanoseconds / 1e9
            max_wall_time = self.get_parameter('max_wall_follow_time').value
            
            if wall_follow_duration > max_wall_time:
                reason = f"UNREACHABLE: Wall following for {wall_follow_duration:.1f}s > {max_wall_time:.1f}s"
                return (True, reason)
        
        return (False, "")
    
    def track_state_change(self, new_state):
        """Rastrea cambios de estado para detección de unreachable"""
        if new_state != self.last_bug2_state:
            self.state_changes += 1
            
            # Tracking de tiempo en WALL_FOLLOWING
            if new_state == self.WALL_FOLLOWING:
                self.wall_follow_start_time = self.get_clock().now()
            elif self.last_bug2_state == self.WALL_FOLLOWING:
                self.wall_follow_start_time = None
            
            self.last_bug2_state = new_state
    
    # ============= COMPORTAMIENTOS BUG2 (ORIGINALES CON TRACKING) =============
    
    def motion_to_goal_behavior(self, position, orientation, sensor_readings):
        """Comportamiento motion-to-goal"""
        angle_to_goal = self.angle_to_goal(position, orientation)
        
        if self.is_path_to_goal_clear(sensor_readings, angle_to_goal):
            forward_speed = self.max_linear_speed * self.forward_speed_ratio
            angular_speed = angle_to_goal * self.angular_gain
            
            left_speed = forward_speed - (angular_speed * self.wheel_separation / 2.0)
            right_speed = forward_speed + (angular_speed * self.wheel_separation / 2.0)
            
            return left_speed, right_speed
        else:
            # Cambio de estado detectado
            self.track_state_change(self.WALL_FOLLOWING)
            self.current_state = self.WALL_FOLLOWING
            
            self.hit_point = position.copy()
            self.hit_distance_to_goal = self.distance_to_goal(position)
            
            left_clear = self.get_clearance_in_direction(sensor_readings, 'left')
            right_clear = self.get_clearance_in_direction(sensor_readings, 'right')
            self.wall_following_side = 'left' if left_clear > right_clear else 'right'
            
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'BUG2: OBSTÁCULO DETECTADO (cambio #{self.state_changes})')
            self.get_logger().info(f'Hit point: ({self.hit_point[0]:.2f}, {self.hit_point[1]:.2f})')
            self.get_logger().info(f'Distancia a meta en hit: {self.hit_distance_to_goal:.2f} m')
            self.get_logger().info(f'Lado de seguimiento: {self.wall_following_side.upper()}')
            self.get_logger().info('=' * 50)
            
            return self.wall_following_behavior(position, orientation, sensor_readings)
    
    def can_leave_wall_following(self, position, orientation, sensor_readings):
        """Condiciones Bug2 para dejar wall following"""
        if not self.is_on_m_line(position):
            return False
        
        current_distance = self.distance_along_m_line_to_goal(position)
        if current_distance >= self.hit_distance_to_goal:
            return False
        
        angle_to_goal = self.angle_to_goal(position, orientation)
        if not self.is_path_to_goal_clear(sensor_readings, angle_to_goal):
            return False
        
        return True
    
    def wall_following_behavior(self, position, orientation, sensor_readings):
        """Comportamiento wall following"""
        if self.can_leave_wall_following(position, orientation, sensor_readings):
            # Cambio de estado detectado
            self.track_state_change(self.MOTION_TO_GOAL)
            self.current_state = self.MOTION_TO_GOAL
            
            current_dist = self.distance_along_m_line_to_goal(position)
            
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'BUG2: DEJANDO WALL_FOLLOWING (cambio #{self.state_changes})')
            self.get_logger().info(f'Distancia a meta: {current_dist:.2f} m (hit: {self.hit_distance_to_goal:.2f} m)')
            self.get_logger().info(f'Mejora: {self.hit_distance_to_goal - current_dist:.2f} m')
            self.get_logger().info('=' * 50)
            
            return self.motion_to_goal_behavior(position, orientation, sensor_readings)
        
        return self.wall_following_control(sensor_readings)
    
    def wall_following_control(self, sensor_readings):
        """Control básico de seguimiento de pared"""
        if self.wall_following_side == 'right':
            wall_sensors = ['sonar_6', 'sonar_7', 'sonar_8']
            front_sensors = ['sonar_4', 'sonar_5']
        else:
            wall_sensors = ['sonar_1', 'sonar_2', 'sonar_3']
            front_sensors = ['sonar_4', 'sonar_5']
        
        wall_distances = []
        for sensor in wall_sensors:
            if sensor in sensor_readings and sensor_readings[sensor]['valid']:
                wall_distances.append(sensor_readings[sensor]['distance'])
        
        front_distances = []
        for sensor in front_sensors:
            if sensor in sensor_readings and sensor_readings[sensor]['valid']:
                front_distances.append(sensor_readings[sensor]['distance'])
        
        avg_wall_distance = sum(wall_distances) / len(wall_distances) if wall_distances else 1.5
        min_front_distance = min(front_distances) if front_distances else 1.5
        
        base_speed = self.wall_follow_speed
        
        if min_front_distance < self.obstacle_threshold:
            if self.wall_following_side == 'right':
                left_speed = -0.5
                right_speed = base_speed
            else:
                left_speed = base_speed
                right_speed = -0.5
        elif avg_wall_distance < self.target_wall_distance - self.wall_distance_tolerance:
            if self.wall_following_side == 'right':
                left_speed = base_speed * 0.6
                right_speed = base_speed * 1.3
            else:
                left_speed = base_speed * 1.3
                right_speed = base_speed * 0.6
        elif avg_wall_distance > self.target_wall_distance + self.wall_distance_tolerance:
            if self.wall_following_side == 'right':
                left_speed = base_speed * 1.3
                right_speed = base_speed * 0.6
            else:
                left_speed = base_speed * 0.6
                right_speed = base_speed * 1.3
        else:
            left_speed = base_speed
            right_speed = base_speed
        
        return left_speed, right_speed
    
    def wheel_speeds_to_twist(self, left_speed, right_speed):
        """Convierte velocidades de ruedas a Twist"""
        linear_vel = (left_speed + right_speed) / 2.0
        angular_vel = (right_speed - left_speed) / self.wheel_separation
        
        linear_vel = max(-self.max_linear_speed, min(self.max_linear_speed, linear_vel))
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        return twist
    
    # ============= LOOP DE CONTROL (ORIGINAL CON UNREACHABLE) =============
    
    def control_loop(self):
        """Loop principal de control Bug2"""
        self.iteration += 1
        
        # Verificar precondiciones
        if self.current_pose is None:
            if self.iteration % 50 == 0:
                self.get_logger().warn('Esperando pose del robot...')
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        if self.goal_point is None:
            if self.iteration % 50 == 0:
                self.get_logger().warn('Esperando meta...')
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        if not self.sonar_readings:
            if self.iteration % 50 == 0:
                self.get_logger().warn('Esperando lecturas de sonares...')
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        position = self.current_position
        orientation = self.current_orientation
        sensor_readings = self.get_sonar_readings_dict()
        
        # Verificar si se alcanzó la meta
        distance_to_goal = self.distance_to_goal(position)
        if distance_to_goal < self.goal_reached_tolerance:
            if self.current_state != self.REACHED:
                self.current_state = self.REACHED
                
                self.get_logger().info('=' * 50)
                self.get_logger().info(f'META ALCANZADA! Distancia: {distance_to_goal:.2f} m')
                self.get_logger().info('Esperando nueva meta...')
                self.get_logger().info('=' * 50)
                
                self.publish_feedback(f"REACHED: d={distance_to_goal:.3f}m")
            
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        # Si estaba REACHED pero ahora está lejos, reanudar
        if self.current_state == self.REACHED:
            self.get_logger().info('🔄 Robot alejado de meta REACHED, reanudando')
            self.current_state = self.MOTION_TO_GOAL
            self.last_bug2_state = self.MOTION_TO_GOAL
        
        # Verificar unreachable
        if not self.unreachable_reported:
            is_unreachable, reason = self.check_unreachable()
            if is_unreachable:
                self.unreachable_reported = True
                self.publish_feedback(reason)
                self.get_logger().warn('⚠ ' + reason)
                self.get_logger().warn('⚠ Continuando navegación...')
        
        # Ejecutar comportamiento según estado
        if self.current_state == self.MOTION_TO_GOAL:
            left_speed, right_speed = self.motion_to_goal_behavior(
                position, orientation, sensor_readings
            )
        elif self.current_state == self.WALL_FOLLOWING:
            left_speed, right_speed = self.wall_following_behavior(
                position, orientation, sensor_readings
            )
        else:
            self.get_logger().warn(f'⚠️ Estado desconocido: {self.current_state}, deteniendo')
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        cmd_vel = self.wheel_speeds_to_twist(left_speed, right_speed)
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Debug periódico
        if self.iteration % self.debug_log_frequency == 0:
            state_names = {
                self.MOTION_TO_GOAL: "MOTION_TO_GOAL",
                self.WALL_FOLLOWING: "WALL_FOLLOWING",
                self.REACHED: "REACHED"
            }
            state_name = state_names.get(self.current_state, "UNKNOWN")
            on_m_line = self.is_on_m_line(position)
            
            self.get_logger().info('-' * 50)
            self.get_logger().info(f'Bug2 Status (iter {self.iteration}):')
            self.get_logger().info(f'  Estado: {state_name}')
            self.get_logger().info(f'  Posición: ({position[0]:.2f}, {position[1]:.2f})')
            self.get_logger().info(f'  Distancia a meta: {distance_to_goal:.2f} m')
            self.get_logger().info(f'  Distancia recorrida: {self.total_distance_traveled:.2f} m')
            self.get_logger().info(f'  Cambios de estado: {self.state_changes}')
            self.get_logger().info(f'  En M-line: {on_m_line}')
            if self.unreachable_reported:
                self.get_logger().info(f'  Unreachable: REPORTADO')
            self.get_logger().info('-' * 50)
    
    # ============= NUEVA: FEEDBACK =============
    
    def publish_feedback(self, message: str):
        """Publica mensaje de feedback con información del objetivo"""
        # Añadir información del objetivo al mensaje
        if self.goal_point is not None:
            goal_str = f"[Goal: ({self.goal_point[0]:.2f}, {self.goal_point[1]:.2f})] "
            full_message = goal_str + message
        else:
            full_message = "[Goal: None] " + message
        
        msg = String()
        msg.data = full_message
        self.feedback_pub.publish(msg)
        self.get_logger().info(f'FEEDBACK: {full_message}')
    
    def publish_periodic_feedback(self):
        """Publica feedback periódico con el estado actual"""
        if self.current_state == self.REACHED or self.goal_point is None:
            return
        
        distance = self.distance_to_goal(self.current_position)
        elapsed_time = 0.0
        
        if self.goal_start_time is not None:
            elapsed_time = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        
        state_names = {
            self.MOTION_TO_GOAL: "MOTION_TO_GOAL",
            self.WALL_FOLLOWING: "WALL_FOLLOWING",
            self.REACHED: "REACHED"
        }
        state_name = state_names.get(self.current_state, "UNKNOWN")
        
        # Añadir indicador si ya se reportó unreachable
        status_suffix = " (UNREACHABLE)" if self.unreachable_reported else ""
        
        feedback_msg = (
            f"STATE: {state_name}{status_suffix} | "
            f"Distance: {distance:.2f}m | "
            f"Traveled: {self.total_distance_traveled:.1f}m | "
            f"Time: {elapsed_time:.1f}s | "
            f"Changes: {self.state_changes}"
        )
        
        self.publish_feedback(feedback_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = Bug2ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo Bug2 Controller...')
    finally:
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()