#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
import math


class Bug2ControllerNode(Node):
    def __init__(self):
        super().__init__('bug2_controller_node')
        
        # ============= PARÁMETROS CONFIGURABLES =============
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('wheel_separation', 0.33)  # metros (ajustar según robot real)
        self.declare_parameter('control_frequency', 20.0)  # Hz
        
        # Parámetros Bug2
        self.declare_parameter('m_line_tolerance', 0.3)  # tolerancia M-line (m)
        self.declare_parameter('goal_reached_tolerance', 0.8)  # tolerancia meta (m)
        self.declare_parameter('obstacle_threshold', 0.4)  # umbral detección obstáculo (m)
        self.declare_parameter('target_wall_distance', 0.5)  # distancia objetivo a pared (m)
        self.declare_parameter('wall_distance_tolerance', 0.2)  # tolerancia pared (m)
        
        # Parámetros de control
        self.declare_parameter('angular_gain', 1.5)  # ganancia control angular
        self.declare_parameter('forward_speed_ratio', 0.8)  # ratio velocidad adelante
        self.declare_parameter('wall_follow_speed', 1.4)  # velocidad wall following
        
        # Logging
        self.declare_parameter('debug_log_frequency', 200)  # cada N iteraciones
        
        # Obtener parámetros
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
        
        # ============= ESTADOS BUG2 =============
        self.MOTION_TO_GOAL = 0
        self.WALL_FOLLOWING = 1
        self.current_state = self.MOTION_TO_GOAL
        
        # ============= VARIABLES BUG2 =============
        self.start_point = None  # Punto inicial para M-line
        self.goal_point = None  # Meta actual
        self.hit_point = None  # Punto donde encontró obstáculo
        self.hit_distance_to_goal = None  # Distancia a meta cuando hit
        self.wall_following_side = 'right'  # Lado de seguimiento
        
        # Estado actual del robot
        self.current_pose = None
        self.current_position = [0.0, 0.0]
        self.current_orientation = 0.0
        
        # Lecturas de sonares (diccionario sensor_id -> distance)
        self.sonar_readings = {}
        self.num_sonars = 16
        
        # Mapeo de ángulos de sensores (mismo que minimal.py)
        self.sensor_angles = {
            1: -90, 2: -50, 3: -30, 4: -10, 5: 10, 6: 30, 7: 50, 8: 90,
            9: 90, 10: 130, 11: 150, 12: 170, 13: -170, 14: -150, 15: -130, 16: -90
        }
        
        # Contador de iteraciones
        self.iteration = 0
        
        # ============= QoS =============
        # QoS para goal (TRANSIENT_LOCAL para recibir última meta)
        goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # ============= PUBLISHERS =============
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
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
        
        # ============= TIMER DE CONTROL =============
        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('=== Bug2 Controller Node Iniciado ===')
        self.get_logger().info(f'Frecuencia de control: {self.control_frequency} Hz')
        self.get_logger().info(f'Velocidad máxima lineal: {self.max_linear_speed} m/s')
        self.get_logger().info(f'Velocidad máxima angular: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'M-line tolerance: {self.m_line_tolerance} m')
        self.get_logger().info(f'Goal tolerance: {self.goal_reached_tolerance} m')
        self.get_logger().info('Esperando pose, goal y sonares...')
    
    # ============= CALLBACKS =============
    
    def pose_callback(self, msg):
        """Callback de pose del robot"""
        self.current_pose = msg
        self.current_position = [msg.pose.position.x, msg.pose.position.y]
        
        # Extraer yaw de quaternion
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # Convertir quaternion a yaw
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def goal_callback(self, msg):
        """Callback de meta"""
        new_goal = [msg.pose.position.x, msg.pose.position.y]
        
        # Si es una meta nueva, resetear Bug2
        if self.goal_point is None or new_goal != self.goal_point:
            self.goal_point = new_goal
            
            # Establecer start_point para M-line
            if self.current_position is not None:
                self.start_point = self.current_position.copy()
            
            # Resetear estado Bug2
            self.current_state = self.MOTION_TO_GOAL
            self.hit_point = None
            self.hit_distance_to_goal = None
            
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'NUEVA META RECIBIDA: ({new_goal[0]:.2f}, {new_goal[1]:.2f})')
            self.get_logger().info(f'Start point (M-line): ({self.start_point[0]:.2f}, {self.start_point[1]:.2f})')
            self.get_logger().info(f'Estado: MOTION_TO_GOAL')
            self.get_logger().info('=' * 50)
    
    def sonar_callback(self, msg, sensor_id):
        """Callback de sensores sonar"""
        self.sonar_readings[sensor_id] = msg.range
    
    # ============= FUNCIONES AUXILIARES =============
    
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
        
        # Normalizar ángulo [-pi, pi]
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
        
        # Si start y goal son muy cercanos, considerar siempre en M-line
        line_length = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        if line_length < 0.1:
            return True
        
        # Distancia perpendicular del punto a la línea
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
        
        # Proyección escalar
        projection = (point_vec[0] * line_vec[0] + point_vec[1] * line_vec[1]) / (line_length**2)
        
        # Punto proyectado sobre la línea
        proj_x = x1 + projection * line_vec[0]
        proj_y = y1 + projection * line_vec[1]
        
        # Distancia del punto proyectado a la meta
        return math.sqrt((x2 - proj_x)**2 + (y2 - proj_y)**2)
    
    def get_sonar_readings_dict(self):
        """Convierte lecturas de sonares a formato similar a minimal.py"""
        readings = {}
        for sensor_id in range(1, self.num_sonars + 1):
            distance = self.sonar_readings.get(sensor_id, 1.5)  # Default 1.5 si no hay lectura
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
        
        # Verificar sensores frontales
        for sensor_name in front_sensors:
            if sensor_name in sensor_readings:
                reading = sensor_readings[sensor_name]
                if reading['valid'] and reading['distance'] < self.obstacle_threshold:
                    return False
        
        # Verificar sensores en dirección de la meta
        for sensor_name, reading in sensor_readings.items():
            if not reading['valid']:
                continue
            
            sensor_angle = math.radians(reading['angle'])
            angle_diff = abs(sensor_angle - angle_to_goal)
            
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
            
            # Si el sensor apunta hacia la meta (±45 grados)
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
    
    # ============= COMPORTAMIENTOS BUG2 =============
    
    def motion_to_goal_behavior(self, position, orientation, sensor_readings):
        """Comportamiento motion-to-goal"""
        angle_to_goal = self.angle_to_goal(position, orientation)
        
        # Verificar si el camino está libre
        if self.is_path_to_goal_clear(sensor_readings, angle_to_goal):
            # Moverse hacia la meta
            forward_speed = self.max_linear_speed * self.forward_speed_ratio
            angular_speed = angle_to_goal * self.angular_gain
            
            # Convertir a velocidades de ruedas (aproximado)
            left_speed = forward_speed - (angular_speed * self.wheel_separation / 2.0)
            right_speed = forward_speed + (angular_speed * self.wheel_separation / 2.0)
            
            return left_speed, right_speed
        else:
            # Obstáculo encontrado - cambiar a wall following
            self.current_state = self.WALL_FOLLOWING
            self.hit_point = position.copy()
            self.hit_distance_to_goal = self.distance_to_goal(position)
            
            # Determinar lado de seguimiento
            left_clear = self.get_clearance_in_direction(sensor_readings, 'left')
            right_clear = self.get_clearance_in_direction(sensor_readings, 'right')
            self.wall_following_side = 'left' if left_clear > right_clear else 'right'
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('BUG2: OBSTÁCULO DETECTADO - Iniciando WALL_FOLLOWING')
            self.get_logger().info(f'Hit point: ({self.hit_point[0]:.2f}, {self.hit_point[1]:.2f})')
            self.get_logger().info(f'Distancia a meta en hit: {self.hit_distance_to_goal:.2f} m')
            self.get_logger().info(f'Lado de seguimiento: {self.wall_following_side.upper()}')
            self.get_logger().info(f'Clearance izq: {left_clear:.2f}, der: {right_clear:.2f}')
            self.get_logger().info('=' * 50)
            
            return self.wall_following_behavior(position, orientation, sensor_readings)
    
    def can_leave_wall_following(self, position, orientation, sensor_readings):
        """Condiciones Bug2 para dejar wall following"""
        # Condición 1: Debe estar en la M-line
        if not self.is_on_m_line(position):
            return False
        
        # Condición 2: Debe estar más cerca de la meta que en el hit point
        current_distance = self.distance_along_m_line_to_goal(position)
        if current_distance >= self.hit_distance_to_goal:
            return False
        
        # Condición 3: El camino directo a la meta debe estar libre
        angle_to_goal = self.angle_to_goal(position, orientation)
        if not self.is_path_to_goal_clear(sensor_readings, angle_to_goal):
            return False
        
        return True
    
    def wall_following_behavior(self, position, orientation, sensor_readings):
        """Comportamiento wall following"""
        # Verificar condiciones de salida Bug2
        if self.can_leave_wall_following(position, orientation, sensor_readings):
            self.current_state = self.MOTION_TO_GOAL
            
            current_dist = self.distance_along_m_line_to_goal(position)
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('BUG2: DEJANDO WALL_FOLLOWING - Volviendo a MOTION_TO_GOAL')
            self.get_logger().info(f'Posición actual: ({position[0]:.2f}, {position[1]:.2f})')
            self.get_logger().info(f'En M-line: TRUE')
            self.get_logger().info(f'Distancia a meta: {current_dist:.2f} m (hit: {self.hit_distance_to_goal:.2f} m)')
            self.get_logger().info(f'Mejora: {self.hit_distance_to_goal - current_dist:.2f} m')
            self.get_logger().info('=' * 50)
            
            return self.motion_to_goal_behavior(position, orientation, sensor_readings)
        
        # Continuar siguiendo la pared
        return self.wall_following_control(sensor_readings)
    
    def wall_following_control(self, sensor_readings):
        """Control básico de seguimiento de pared"""
        if self.wall_following_side == 'right':
            wall_sensors = ['sonar_6', 'sonar_7', 'sonar_8']
            front_sensors = ['sonar_4', 'sonar_5']
        else:
            wall_sensors = ['sonar_1', 'sonar_2', 'sonar_3']
            front_sensors = ['sonar_4', 'sonar_5']
        
        # Obtener distancias a la pared y al frente
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
        
        # Control de seguimiento de pared (velocidades de ruedas)
        base_speed = self.wall_follow_speed
        
        # Obstáculo frontal - girar
        if min_front_distance < self.obstacle_threshold:
            if self.wall_following_side == 'right':
                left_speed = -0.5
                right_speed = base_speed
            else:
                left_speed = base_speed
                right_speed = -0.5
        
        # Muy cerca - alejarse
        elif avg_wall_distance < self.target_wall_distance - self.wall_distance_tolerance:
            if self.wall_following_side == 'right':
                left_speed = base_speed * 0.6
                right_speed = base_speed * 1.3
            else:
                left_speed = base_speed * 1.3
                right_speed = base_speed * 0.6
        
        # Muy lejos - acercarse
        elif avg_wall_distance > self.target_wall_distance + self.wall_distance_tolerance:
            if self.wall_following_side == 'right':
                left_speed = base_speed * 1.3
                right_speed = base_speed * 0.6
            else:
                left_speed = base_speed * 0.6
                right_speed = base_speed * 1.3
        
        # Distancia correcta - avanzar
        else:
            left_speed = base_speed
            right_speed = base_speed
        
        return left_speed, right_speed
    
    def wheel_speeds_to_twist(self, left_speed, right_speed):
        """Convierte velocidades de ruedas a Twist"""
        # Aproximación cinemática diferencial
        linear_vel = (left_speed + right_speed) / 2.0
        angular_vel = (right_speed - left_speed) / self.wheel_separation
        
        # Limitar velocidades
        linear_vel = max(-self.max_linear_speed, min(self.max_linear_speed, linear_vel))
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        return twist
    
    # ============= LOOP DE CONTROL =============
    
    def control_loop(self):
        """Loop principal de control Bug2"""
        self.iteration += 1
        
        # Verificar que tenemos datos necesarios
        if self.current_pose is None:
            if self.iteration % 50 == 0:
                self.get_logger().warn('Esperando pose del robot...')
            return
        
        if self.goal_point is None:
            if self.iteration % 50 == 0:
                self.get_logger().warn('Esperando meta...')
            return
        
        if not self.sonar_readings:
            if self.iteration % 50 == 0:
                self.get_logger().warn('Esperando lecturas de sonares...')
            return
        
        # Obtener estado actual
        position = self.current_position
        orientation = self.current_orientation
        sensor_readings = self.get_sonar_readings_dict()
        
        # Verificar si se alcanzó la meta
        distance_to_goal = self.distance_to_goal(position)
        if distance_to_goal < self.goal_reached_tolerance:
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'META ALCANZADA! Distancia: {distance_to_goal:.2f} m')
            self.get_logger().info('Esperando nueva meta del goal_manager...')
            self.get_logger().info('=' * 50)
            
            # Detener robot
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            return
        
        # Ejecutar comportamiento según estado Bug2
        if self.current_state == self.MOTION_TO_GOAL:
            left_speed, right_speed = self.motion_to_goal_behavior(
                position, orientation, sensor_readings
            )
        else:  # WALL_FOLLOWING
            left_speed, right_speed = self.wall_following_behavior(
                position, orientation, sensor_readings
            )
        
        # Convertir a Twist y publicar
        cmd_vel = self.wheel_speeds_to_twist(left_speed, right_speed)
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Debug periódico
        if self.iteration % self.debug_log_frequency == 0:
            state_name = "MOTION_TO_GOAL" if self.current_state == self.MOTION_TO_GOAL else "WALL_FOLLOWING"
            on_m_line = self.is_on_m_line(position)
            
            self.get_logger().info('-' * 50)
            self.get_logger().info(f'Bug2 Status (iter {self.iteration}):')
            self.get_logger().info(f'  Estado: {state_name}')
            self.get_logger().info(f'  Posición: ({position[0]:.2f}, {position[1]:.2f})')
            self.get_logger().info(f'  Meta: ({self.goal_point[0]:.2f}, {self.goal_point[1]:.2f})')
            self.get_logger().info(f'  Distancia a meta: {distance_to_goal:.2f} m')
            self.get_logger().info(f'  En M-line: {on_m_line}')
            self.get_logger().info(f'  Cmd_vel: linear={cmd_vel.linear.x:.2f}, angular={cmd_vel.angular.z:.2f}')
            self.get_logger().info('-' * 50)


def main(args=None):
    rclpy.init(args=args)
    
    node = Bug2ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo Bug2 Controller...')
    finally:
        # Detener robot
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()