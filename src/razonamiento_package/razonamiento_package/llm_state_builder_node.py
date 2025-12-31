#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Range
from visualization_msgs.msg import MarkerArray
import json
import math


class LLMStateBuilderNode(Node):
    def __init__(self):
        super().__init__('llm_state_builder_node')
        
        # ============= PARÁMETROS =============
        self.declare_parameter('compact_precision', 2)  # Decimales para floats
        self.declare_parameter('include_covariance', False)  # Incluir covarianza de pose
        self.declare_parameter('max_frontiers', 5)  # Máximo número de frontiers a incluir
        self.declare_parameter('max_graph_nodes', 20)  # Máximo número de nodos del grafo
        self.declare_parameter('sonar_threshold', 1.5)  # Distancia máxima relevante de sonares
        
        # Obtener parámetros
        self.precision = int(self.get_parameter('compact_precision').value)
        self.include_covariance = bool(self.get_parameter('include_covariance').value)
        self.max_frontiers = int(self.get_parameter('max_frontiers').value)
        self.max_graph_nodes = int(self.get_parameter('max_graph_nodes').value)
        self.sonar_threshold = float(self.get_parameter('sonar_threshold').value)
        
        # ============= ESTADO DEL MUNDO =============
        self.current_pose = None
        self.sonar_readings = {}  # {sensor_id: distance}
        self.frontiers = []
        self.graph_nodes = []
        self.graph_edges = []
        
        # Ángulos y etiquetas de los sonares (8 sensores, 20° entre ellos)
        # Sensor 1 = izquierda extrema, Sensor 8 = derecha extrema
        self.sonar_labels = {
            1: 'L',    # Left (izquierda extrema)
            2: 'FL',   # Front-Left
            3: 'FL',   # Front-Left
            4: 'F',    # Front
            5: 'F',    # Front
            6: 'FR',   # Front-Right
            7: 'FR',   # Front-Right
            8: 'R'     # Right (derecha extrema)
        }
        
        # ============= SUBSCRIBERS =============
        # Pose del robot
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )
        
        # Sonares (1-8)
        self.sonar_subs = []
        for i in range(1, 9):
            sub = self.create_subscription(
                Range,
                f'/robot/sonar_{i}',
                lambda msg, sensor_id=i: self.sonar_callback(msg, sensor_id),
                10
            )
            self.sonar_subs.append(sub)
        
        # Frontiers
        self.frontier_sub = self.create_subscription(
            MarkerArray,
            '/frontier_markers',
            self.frontier_callback,
            10
        )
        
        # Grafo topológico
        self.graph_sub = self.create_subscription(
            MarkerArray,
            '/topological_graph_markers',
            self.graph_callback,
            10
        )
        
        # ============= COMUNICACIÓN ON-DEMAND =============
        # Subscriber: recibe solicitudes de estado
        self.request_sub = self.create_subscription(
            String,
            '/llm/state_request',
            self.state_request_callback,
            10
        )
        
        # Publisher: publica estado en JSON
        self.state_pub = self.create_publisher(
            String,
            '/llm/world_state',
            10
        )
        
        # ============= ESTADÍSTICAS =============
        self.requests_count = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('LLM State Builder Node Started')
        self.get_logger().info(f'Precision: {self.precision} decimals')
        self.get_logger().info(f'Max frontiers: {self.max_frontiers}')
        self.get_logger().info(f'Max graph nodes: {self.max_graph_nodes}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Listening on: /llm/state_request')
        self.get_logger().info('Publishing to: /llm/world_state')
        self.get_logger().info('=' * 60)
    
    # ============= CALLBACKS DE DATOS =============
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback de pose del robot"""
        self.current_pose = msg
    
    def sonar_callback(self, msg: Range, sensor_id: int):
        """Callback de sensores sonar - guarda TODAS las lecturas"""
        # Guardar lectura, si supera umbral o es inválida, usar valor máximo
        if math.isinf(msg.range) or math.isnan(msg.range) or msg.range >= self.sonar_threshold:
            self.sonar_readings[sensor_id] = self.sonar_threshold
        else:
            self.sonar_readings[sensor_id] = msg.range
    
    def frontier_callback(self, msg: MarkerArray):
        """Callback de frontiers"""
        self.frontiers = []
        
        for marker in msg.markers:
            # Solo procesar markers de centroids
            if marker.ns == "frontier_centroids":
                frontier = {
                    'id': marker.id,
                    'x': round(marker.pose.position.x, self.precision),
                    'y': round(marker.pose.position.y, self.precision),
                    # Extraer tamaño del scale (proporcional al área)
                    'size': round(marker.scale.x * 100, 0)  # Aproximación
                }
                self.frontiers.append(frontier)
        
        # Ordenar por tamaño (más grandes primero) y limitar
        self.frontiers.sort(key=lambda f: f['size'], reverse=True)
        self.frontiers = self.frontiers[:self.max_frontiers]
    
    def graph_callback(self, msg: MarkerArray):
        """Callback del grafo topológico"""
        self.graph_nodes = []
        self.graph_edges = []
        
        # Primero procesar nodos
        node_positions = {}  # {node_id: (x, y, type)}
        
        for marker in msg.markers:
            # Etiquetas de nodos (TEXT_VIEW_FACING)
            if marker.ns == "node_labels":
                node_id = marker.text
                x = round(marker.pose.position.x, self.precision)
                y = round(marker.pose.position.y, self.precision)
                
                # Intentar determinar tipo por el nombre
                if node_id.startswith('m') or '_' in node_id:
                    node_type = 'junction'  # Merged nodes o con zona
                elif node_id.startswith('t'):
                    node_type = 'turn'
                elif node_id.startswith('n'):
                    node_type = 'endpoint'
                else:
                    node_type = 'unknown'
                
                node_positions[node_id] = (x, y, node_type)
        
        # Procesar aristas para obtener conectividad
        edges_list = []
        for marker in msg.markers:
            if marker.ns == "edges":
                # LINE_LIST: cada par de puntos es una arista
                points = marker.points
                for i in range(0, len(points), 2):
                    if i + 1 < len(points):
                        p1 = points[i]
                        p2 = points[i + 1]
                        
                        x1 = round(p1.x, self.precision)
                        y1 = round(p1.y, self.precision)
                        x2 = round(p2.x, self.precision)
                        y2 = round(p2.y, self.precision)
                        
                        edges_list.append(((x1, y1), (x2, y2)))
        
        # Construir estructura de grafo con vecinos
        for node_id, (x, y, node_type) in node_positions.items():
            # Encontrar vecinos (nodos conectados por aristas)
            neighbors = []
            
            for (x1, y1), (x2, y2) in edges_list:
                # Si este nodo es uno de los extremos de la arista
                if abs(x - x1) < 0.01 and abs(y - y1) < 0.01:
                    # Buscar el nodo en el otro extremo
                    for other_id, (ox, oy, _) in node_positions.items():
                        if abs(ox - x2) < 0.01 and abs(oy - y2) < 0.01:
                            neighbors.append(other_id)
                            break
                elif abs(x - x2) < 0.01 and abs(y - y2) < 0.01:
                    # Buscar el nodo en el otro extremo
                    for other_id, (ox, oy, _) in node_positions.items():
                        if abs(ox - x1) < 0.01 and abs(oy - y1) < 0.01:
                            neighbors.append(other_id)
                            break
            
            node = {
                'id': node_id,
                'x': x,
                'y': y,
                'type': node_type,
                'neighbors': neighbors
            }
            self.graph_nodes.append(node)
        
        # Limitar número de nodos (ordenar por relevancia si es necesario)
        self.graph_nodes = self.graph_nodes[:self.max_graph_nodes]
    
    # ============= GENERACIÓN DE ESTADO JSON =============
    
    def state_request_callback(self, msg: String):
        """Callback cuando se solicita el estado del mundo"""
        self.requests_count += 1
        
        # Extraer ID del request (si viene vacío, usar contador)
        request_id = msg.data.strip() if msg.data.strip() else str(self.requests_count)
        
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'State request received: id="{request_id}"')
        
        # Generar JSON del estado con el ID proporcionado
        world_state_json = self.build_world_state_json(request_id)
        
        # Publicar
        response_msg = String()
        response_msg.data = world_state_json
        self.state_pub.publish(response_msg)
        
        json_size = len(world_state_json)
        self.get_logger().info(f'World state published: {json_size} chars, id="{request_id}"')
        self.get_logger().info(f'Preview: {world_state_json[:100]}...')
        self.get_logger().info('-' * 60)
    
    def build_world_state_json(self, request_id: str) -> str:
        """Construye el JSON compacto (nombres legibles) del estado del mundo"""
        
        state = {}
        
        # ===== POSE DEL ROBOT =====
        if self.current_pose is not None:
            pose = self.current_pose.pose.pose
            
            x = round(pose.position.x, self.precision)
            y = round(pose.position.y, self.precision)
            
            # Orientación (yaw en grados)
            q = pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            yaw_deg = round(math.degrees(yaw), 0)
            
            state['robot'] = {
                'x': x,
                'y': y,
                'yaw': int(yaw_deg)
            }
        else:
            state['robot'] = None
        
        # ===== SONARES (formato agrupado por dirección) =====
        # Agrupar por etiqueta y usar el valor MÍNIMO de cada grupo
        sonar_groups = {'left': [], 'front_left': [], 'front': [], 'front_right': [], 'right': []}
        
        for sensor_id in range(1, 9):
            label = self.sonar_labels.get(sensor_id, 'U')
            distance = self.sonar_readings.get(sensor_id, self.sonar_threshold)
            
            # Mapear etiquetas cortas a nombres completos
            if label == 'L':
                sonar_groups['left'].append(distance)
            elif label == 'FL':
                sonar_groups['front_left'].append(distance)
            elif label == 'F':
                sonar_groups['front'].append(distance)
            elif label == 'FR':
                sonar_groups['front_right'].append(distance)
            elif label == 'R':
                sonar_groups['right'].append(distance)
        
        # Compactar: tomar mínimo de cada grupo
        state['sonars'] = {
            'left': round(min(sonar_groups['left']), self.precision) if sonar_groups['left'] else self.sonar_threshold,
            'front_left': round(min(sonar_groups['front_left']), self.precision) if sonar_groups['front_left'] else self.sonar_threshold,
            'front': round(min(sonar_groups['front']), self.precision) if sonar_groups['front'] else self.sonar_threshold,
            'front_right': round(min(sonar_groups['front_right']), self.precision) if sonar_groups['front_right'] else self.sonar_threshold,
            'right': round(min(sonar_groups['right']), self.precision) if sonar_groups['right'] else self.sonar_threshold
        }
        
        # ===== FRONTIERS =====
        if self.frontiers:
            state['frontiers'] = [
                {
                    'x': f['x'],
                    'y': f['y'],
                    'size': int(f['size'])
                }
                for f in self.frontiers
            ]
        else:
            state['frontiers'] = []
        
        # ===== GRAFO TOPOLÓGICO CON VECINOS =====
        if self.graph_nodes:
            state['graph'] = [
                {
                    'id': n['id'],
                    'x': n['x'],
                    'y': n['y'],
                    'type': n['type'],
                    'neighbors': n['neighbors']
                }
                for n in self.graph_nodes
            ]
        else:
            state['graph'] = []
        
        # ===== METADATOS =====
        state['meta'] = {
            'timestamp': self.get_clock().now().nanoseconds // 1000000,
            'request_id': request_id
        }
        
        # Convertir a JSON compacto (sin espacios)
        return json.dumps(state, separators=(',', ':'))
    
    def get_state_summary(self) -> str:
        """Obtiene un resumen legible del estado (para debugging)"""
        summary = []
        summary.append(f"Pose: {'OK' if self.current_pose else 'None'}")
        summary.append(f"Sonars: {len(self.sonar_readings)}/8")
        summary.append(f"Frontiers: {len(self.frontiers)}")
        summary.append(f"Graph nodes: {len(self.graph_nodes)}")
        return " | ".join(summary)


def main(args=None):
    rclpy.init(args=args)
    
    node = LLMStateBuilderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM State Builder Node...')
    finally:
        node.get_logger().info('=' * 60)
        node.get_logger().info('Final Statistics:')
        node.get_logger().info(f'  Total requests: {node.requests_count}')
        node.get_logger().info(f'  Final state: {node.get_state_summary()}')
        node.get_logger().info('=' * 60)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()