#!/usr/bin/env python3
"""
Nodo de Mapeo con Occupancy Grid
Construye un mapa de ocupación usando sensores sonar y pose del robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import math
from PIL import Image
import os
from datetime import datetime


class OccupancyMapperNode(Node):
    """
    Nodo que construye un mapa de ocupación en tiempo real
    
    SUSCRIBE:
        - /robot/pose (PoseStamped)
        - /robot/sonar_* (Range) - 16 sensores
    
    PUBLICA:
        - /map (OccupancyGrid) - Para RViz2
    """
    
    def __init__(self):
        super().__init__('occupancy_mapper_node')
        
        # Parámetros del mapa
        self.declare_parameter('map_width', 10.0)      # metros
        self.declare_parameter('map_height', 10.0)     # metros
        self.declare_parameter('resolution', 0.05)     # metros por celda (5cm)
        self.declare_parameter('origin_x', -5.0)       # esquina inferior izquierda
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('save_interval', 30.0)  # guardar cada 30 segundos
        self.declare_parameter('output_dir', '/ros2_ws/maps')
        
        # Obtener parámetros
        map_width = self.get_parameter('map_width').value
        map_height = self.get_parameter('map_height').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        save_interval = self.get_parameter('save_interval').value
        self.output_dir = self.get_parameter('output_dir').value
        
        # Dimensiones del grid
        self.width = int(map_width / self.resolution)
        self.height = int(map_height / self.resolution)
        
        # Mapa de ocupación (log-odds representation)
        # Inicializar en 0 (probabilidad 0.5 = desconocido)
        self.log_odds_map = np.zeros((self.height, self.width), dtype=np.float32)
        
        # Parámetros del modelo de sensor
        self.l_occupied = 0.85    # Log-odds para celda ocupada
        self.l_free = -0.4        # Log-odds para celda libre
        self.max_range = 5.0      # Rango máximo del sensor
        self.min_range = 0.05     # Rango mínimo
        
        # Estado del robot
        self.current_pose = None
        self.sonar_readings = {}
        
        # Mapeo de ángulos de sensores (del código original)
        self.sensor_angles = {
            1: -90, 2: -50, 3: -30, 4: -10,
            5: 10, 6: 30, 7: 50, 8: 90,
            9: 90, 10: 130, 11: 150, 12: 170,
            13: -170, 14: -150, 15: -130, 16: -90
        }
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )
        
        # Suscribirse a todos los sonares
        for i in range(1, 17):
            self.create_subscription(
                Range,
                f'/robot/sonar_{i}',
                lambda msg, sid=i: self.sonar_callback(msg, sid),
                10
            )
        
        # Publisher del mapa (para RViz2)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Timer para actualizar mapa
        self.update_timer = self.create_timer(0.1, self.update_map_callback)
        
        # Timer para guardar mapa
        self.save_timer = self.create_timer(save_interval, self.save_map)
        
        # Crear directorio de salida
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('Nodo Occupancy Mapper iniciado')
        self.get_logger().info(f'Mapa: {self.width}x{self.height} celdas, res={self.resolution}m')
        self.get_logger().info(f'Guardando en: {self.output_dir}')
    
    def pose_callback(self, msg):
        """Actualiza pose del robot"""
        self.current_pose = msg
    
    def sonar_callback(self, msg, sensor_id):
        """Almacena lectura de sensor"""
        self.sonar_readings[sensor_id] = msg
    
    def world_to_map(self, x, y):
        """
        Convierte coordenadas mundo (metros) a índices del grid
        Returns: (map_x, map_y) o None si está fuera del mapa
        """
        map_x = int((x - self.origin_x) / self.resolution)
        map_y = int((y - self.origin_y) / self.resolution)
        
        if 0 <= map_x < self.width and 0 <= map_y < self.height:
            return (map_x, map_y)
        return None
    
    def map_to_world(self, map_x, map_y):
        """Convierte índices del grid a coordenadas mundo"""
        x = self.origin_x + (map_x + 0.5) * self.resolution
        y = self.origin_y + (map_y + 0.5) * self.resolution
        return (x, y)
    
    def bresenham_line(self, x0, y0, x1, y1):
        """
        Algoritmo de Bresenham para obtener todas las celdas en una línea
        Returns: lista de tuplas (x, y)
        """
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            cells.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return cells
    
    def update_map_callback(self):
        """Actualiza el mapa con las lecturas actuales de sensores"""
        if self.current_pose is None or not self.sonar_readings:
            return
        
        # Posición del robot
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        
        # Orientación del robot (quaternion a yaw)
        q = self.current_pose.pose.orientation
        robot_yaw = 2 * math.atan2(q.z, q.w)
        
        # Convertir posición del robot a grid
        robot_cell = self.world_to_map(robot_x, robot_y)
        if robot_cell is None:
            return
        
        robot_mx, robot_my = robot_cell
        
        # Procesar cada lectura de sensor
        for sensor_id, range_msg in self.sonar_readings.items():
            # Filtrar lecturas inválidas
            if range_msg.range < self.min_range or range_msg.range > self.max_range:
                continue
            
            # Ángulo del sensor en el marco del robot
            sensor_angle_deg = self.sensor_angles.get(sensor_id, 0)
            sensor_angle = math.radians(sensor_angle_deg)
            
            # Ángulo global del rayo
            ray_angle = robot_yaw + sensor_angle
            
            # Punto final del rayo (obstáculo detectado)
            end_x = robot_x + range_msg.range * math.cos(ray_angle)
            end_y = robot_y + range_msg.range * math.sin(ray_angle)
            
            # Convertir a grid
            end_cell = self.world_to_map(end_x, end_y)
            if end_cell is None:
                continue
            
            end_mx, end_my = end_cell
            
            # Obtener todas las celdas atravesadas por el rayo
            ray_cells = self.bresenham_line(robot_mx, robot_my, end_mx, end_my)
            
            # Actualizar celdas libres (todas excepto la última)
            for i, (cx, cy) in enumerate(ray_cells[:-1]):
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    self.log_odds_map[cy, cx] += self.l_free
                    # Saturar para evitar overflow
                    self.log_odds_map[cy, cx] = max(-10, self.log_odds_map[cy, cx])
            
            # Actualizar celda ocupada (la última)
            if len(ray_cells) > 0:
                ox, oy = ray_cells[-1]
                if 0 <= ox < self.width and 0 <= oy < self.height:
                    self.log_odds_map[oy, ox] += self.l_occupied
                    # Saturar
                    self.log_odds_map[oy, ox] = min(10, self.log_odds_map[oy, ox])
        
        # Publicar mapa para RViz2
        self.publish_map()
    
    def log_odds_to_probability(self, log_odds_map):
        """Convierte log-odds a probabilidades [0, 1]"""
        return 1.0 - 1.0 / (1.0 + np.exp(log_odds_map))
    
    def publish_map(self):
        """Publica el mapa como OccupancyGrid para RViz2"""
        # Convertir log-odds a probabilidades
        prob_map = self.log_odds_to_probability(self.log_odds_map)
        
        # Convertir a valores de OccupancyGrid [0, 100] o -1 (desconocido)
        occupancy_data = np.zeros_like(prob_map, dtype=np.int8)
        
        # -1 = desconocido, 0 = libre, 100 = ocupado
        unknown_mask = np.abs(self.log_odds_map) < 0.1
        occupancy_data[unknown_mask] = -1
        occupancy_data[~unknown_mask] = (prob_map[~unknown_mask] * 100).astype(np.int8)
        
        # Crear mensaje OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'world'
        
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # ROS usa row-major order, invertir Y
        grid_msg.data = np.flipud(occupancy_data).flatten().tolist()
        
        self.map_pub.publish(grid_msg)
    
    def save_map(self):
        """Guarda el mapa como imagen PNG y archivo PGM"""
        self.get_logger().info('Guardando mapa...')
        
        # Convertir log-odds a probabilidades
        prob_map = self.log_odds_to_probability(self.log_odds_map)
        
        # Convertir a escala de grises [0, 255]
        # 0 = negro (ocupado), 255 = blanco (libre), 128 = gris (desconocido)
        image_data = np.zeros_like(prob_map, dtype=np.uint8)
        
        # Zonas desconocidas (gris)
        unknown_mask = np.abs(self.log_odds_map) < 0.1
        image_data[unknown_mask] = 128
        
        # Zonas conocidas: invertir (0=ocupado negro, 255=libre blanco)
        image_data[~unknown_mask] = ((1.0 - prob_map[~unknown_mask]) * 255).astype(np.uint8)
        
        # Invertir verticalmente para que coincida con visualización estándar
        image_data = np.flipud(image_data)
        
        # Timestamp para nombre de archivo
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Guardar como PNG
        png_path = os.path.join(self.output_dir, f'map_{timestamp}.png')
        img = Image.fromarray(image_data, mode='L')
        img.save(png_path)
        self.get_logger().info(f'✓ PNG guardado: {png_path}')
        
        # Guardar como PGM (formato ROS estándar)
        pgm_path = os.path.join(self.output_dir, f'map_{timestamp}.pgm')
        self.save_pgm(image_data, pgm_path)
        self.get_logger().info(f'✓ PGM guardado: {pgm_path}')
        
        # Guardar YAML (metadatos para ROS)
        yaml_path = os.path.join(self.output_dir, f'map_{timestamp}.yaml')
        self.save_yaml(pgm_path, yaml_path)
        self.get_logger().info(f'✓ YAML guardado: {yaml_path}')
    
    def save_pgm(self, image_data, filename):
        """Guarda imagen en formato PGM (Portable Gray Map)"""
        with open(filename, 'wb') as f:
            # Header PGM
            f.write(b'P5\n')
            f.write(f'{self.width} {self.height}\n'.encode())
            f.write(b'255\n')
            # Datos binarios
            f.write(image_data.tobytes())
    
    def save_yaml(self, pgm_filename, yaml_filename):
        """Guarda archivo YAML con metadatos del mapa (formato ROS)"""
        yaml_content = f"""image: {os.path.basename(pgm_filename)}
resolution: {self.resolution}
origin: [{self.origin_x}, {self.origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
        with open(yaml_filename, 'w') as f:
            f.write(yaml_content)
    
    def shutdown(self):
        """Guarda el mapa final al cerrar"""
        self.get_logger().info('Guardando mapa final...')
        self.save_map()


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()