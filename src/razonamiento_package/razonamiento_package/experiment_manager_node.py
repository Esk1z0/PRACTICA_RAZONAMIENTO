#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from visualization_msgs.msg import MarkerArray
import yaml
import math
import time


class ExperimentManagerNode(Node):
    def __init__(self):
        super().__init__('experiment_manager_node')
        
        # ============= PARÁMETROS =============
        self.declare_parameter('experiment_file', '')
        self.declare_parameter('goal_distance_threshold', 0.8)  # Distancia para considerar objetivo alcanzado
        self.declare_parameter('check_rate_hz', 5.0)  # Frecuencia de chequeo
        
        # Obtener parámetros
        self.experiment_file = self.get_parameter('experiment_file').value
        self.goal_threshold = float(self.get_parameter('goal_distance_threshold').value)
        self.check_rate = float(self.get_parameter('check_rate_hz').value)
        
        # ============= ESTADO =============
        self.current_pose = None
        self.zones = {}  # {zone_name: {min_x, max_x, min_y, max_y}}
        self.experiment = None
        self.experiment_type = None
        
        # Estado del experimento
        self.goals_completed = []
        self.start_time = None
        self.last_zone_entry_time = {}  # {zone_name: timestamp}
        self.experiment_running = False
        
        # ============= QOS =================
        experiment_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # ← Cambiar a TRANSIENT_LOCAL
            history=HistoryPolicy.KEEP_LAST
        )

        event_qos = QoSProfile(
            depth=100,  # Buffer grande
            reliability=ReliabilityPolicy.RELIABLE,  # ← CRÍTICO: necesita todos los eventos
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ============= PUBLISHERS =============
        self.experiment_pub = self.create_publisher(String, '/experiment', experiment_qos)
        
        # ============= SUBSCRIBERS =============
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )
        
        self.zone_sub = self.create_subscription(
            MarkerArray,
            '/zone_markers',
            self.zone_callback,
            10
        )
        
        # ============= TIMER =============
        self.check_timer = self.create_timer(1.0 / self.check_rate, self.check_experiment_status)
        
        # ============= INICIALIZACIÓN =============
        self.get_logger().info(f'✓ Experiment Manager ready | File: {self.experiment_file or "none"} | Threshold: {self.goal_threshold}m')
        
        # Cargar experimento
        if self.experiment_file and self.experiment_file != '':
            self.load_experiment()
        else:
            self.get_logger().warn('No experiment file - waiting...')
    
    # ============= CALLBACKS =============
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback de pose del robot"""
        self.current_pose = msg
        
        # Iniciar experimento si aún no ha empezado
        if self.experiment and not self.experiment_running and self.zones:
            self.start_experiment()
    
    def zone_callback(self, msg: MarkerArray):
        """Callback de zonas - extraer límites de cada zona"""
        for marker in msg.markers:
            if marker.ns == "zone_labels":
                zone_name = marker.text
                
                # Calcular límites del rectángulo desde el CUBE marker correspondiente
                # Buscar el marker CUBE con el mismo ID
                for m in msg.markers:
                    if m.ns == "zone_rectangles" and m.id == marker.id - 2000:
                        # Centro del cubo
                        center_x = m.pose.position.x
                        center_y = m.pose.position.y
                        
                        # Tamaño del cubo
                        width = m.scale.x
                        height = m.scale.y
                        
                        # Calcular límites
                        self.zones[zone_name] = {
                            'min_x': center_x - width / 2.0,
                            'max_x': center_x + width / 2.0,
                            'min_y': center_y - height / 2.0,
                            'max_y': center_y + height / 2.0
                        }
                        break
        
        if self.zones and not self.experiment_running:
            zone_names = ', '.join(list(self.zones.keys()))
            self.get_logger().info(f'✓ Zones loaded: {zone_names}')
    
    # ============= CARGA DE EXPERIMENTO =============
    
    def load_experiment(self):
        """Carga el experimento desde archivo YAML"""
        try:
            with open(self.experiment_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if 'experiment' not in config:
                self.get_logger().error('YAML missing "experiment" section')
                return
            
            exp = config['experiment']
            self.experiment_type = exp.get('type')
            self.experiment = exp
            
            # Log compacto según tipo
            if self.experiment_type == 'single_goal':
                goal_str = exp.get('goal', {}).get('zone', exp.get('goal', {}))
                self.get_logger().info(f'✓ Experiment loaded: {self.experiment_type} → {goal_str}')
            elif self.experiment_type in ['multigoal_simple', 'multigoal_restricted']:
                num_goals = len(exp.get('goals', []))
                restrictions = []
                if self.experiment_type == 'multigoal_restricted':
                    if exp.get('require_order'):
                        restrictions.append('ORDER')
                    if exp.get('forbidden_zones'):
                        restrictions.append(f'FORBIDDEN:{",".join(exp.get("forbidden_zones", []))}')
                    if exp.get('time_limit'):
                        restrictions.append(f'{exp.get("time_limit")}s')
                
                restriction_str = f' [{", ".join(restrictions)}]' if restrictions else ''
                self.get_logger().info(f'✓ Experiment loaded: {self.experiment_type} → {num_goals} goals{restriction_str}')
            else:
                self.get_logger().info(f'✓ Experiment loaded: {self.experiment_type}')
            
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {self.experiment_file}')
        except yaml.YAMLError as e:
            self.get_logger().error(f'YAML parse error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error loading experiment: {e}')
    
    def start_experiment(self):
        """Inicia el experimento y publica el objetivo inicial"""
        self.experiment_running = True
        self.start_time = time.time()
        self.goals_completed = []
        
        # Generar mensaje inicial según tipo de experimento
        if self.experiment_type == 'single_goal':
            message = self.generate_single_goal_message()
        elif self.experiment_type == 'multigoal_simple':
            message = self.generate_multigoal_simple_message()
        elif self.experiment_type == 'multigoal_restricted':
            message = self.generate_multigoal_restricted_message()
        else:
            message = f"Experimento desconocido: {self.experiment_type}"
        
        self.publish_experiment_message(message)
        self.get_logger().info(f'🚀 EXPERIMENT START | {message[:80]}...')
    
    # ============= GENERACIÓN DE MENSAJES =============
    
    def generate_single_goal_message(self):
        """Genera mensaje para experimento single_goal"""
        goal = self.experiment.get('goal', {})
        description = self.experiment.get('description', '')
        
        if 'zone' in goal:
            return f"{description}. Objetivo: navegar a la zona '{goal['zone']}'."
        elif 'coordinates' in goal:
            x, y = goal['coordinates']
            return f"{description}. Objetivo: navegar a las coordenadas ({x}, {y})."
        else:
            return f"{description}. Objetivo: {goal}"
    
    def generate_multigoal_simple_message(self):
        """Genera mensaje para experimento multigoal_simple"""
        goals = self.experiment.get('goals', [])
        description = self.experiment.get('description', '')
        
        goal_list = []
        for i, goal in enumerate(goals):
            if 'zone' in goal:
                goal_list.append(f"zona '{goal['zone']}'")
            elif 'coordinates' in goal:
                x, y = goal['coordinates']
                goal_list.append(f"coordenadas ({x}, {y})")
        
        goals_str = ', '.join(goal_list)
        return f"{description}. Objetivos (cualquier orden): {goals_str}."
    
    def generate_multigoal_restricted_message(self):
        """Genera mensaje para experimento multigoal_restricted"""
        goals = self.experiment.get('goals', [])
        description = self.experiment.get('description', '')
        require_order = self.experiment.get('require_order', False)
        forbidden_zones = self.experiment.get('forbidden_zones', [])
        time_limit = self.experiment.get('time_limit')
        
        goal_list = []
        for i, goal in enumerate(goals):
            if 'zone' in goal:
                goal_list.append(f"{i+1}. zona '{goal['zone']}'")
            elif 'coordinates' in goal:
                x, y = goal['coordinates']
                goal_list.append(f"{i+1}. coordenadas ({x}, {y})")
        
        goals_str = ', '.join(goal_list)
        
        message = f"{description}. Objetivos"
        if require_order:
            message += " (ORDEN OBLIGATORIO)"
        message += f": {goals_str}."
        
        if forbidden_zones:
            message += f" RESTRICCIÓN: NO entrar en zonas {forbidden_zones}."
        
        if time_limit:
            message += f" Tiempo límite: {time_limit}s."
        
        return message
    
    # ============= VERIFICACIÓN DE OBJETIVOS =============
    
    def check_experiment_status(self):
        """Verifica el estado del experimento"""
        if not self.experiment_running or not self.current_pose:
            return
        
        # Verificar timeout global
        if self.experiment_type == 'multigoal_restricted':
            time_limit = self.experiment.get('time_limit')
            if time_limit:
                elapsed = time.time() - self.start_time
                if elapsed > time_limit:
                    self.finish_experiment(
                        False,
                        f"[TIMEOUT] Tiempo límite de {time_limit}s superado. Experimento no completado en {elapsed:.1f}s."
                    )
                    return
        
        # Verificar zonas prohibidas
        if self.experiment_type == 'multigoal_restricted':
            violation = self.check_forbidden_zones()
            if violation:
                self.finish_experiment(False, violation)
                return
        
        # Verificar cumplimiento de objetivos
        if self.experiment_type == 'single_goal':
            self.check_single_goal()
        elif self.experiment_type == 'multigoal_simple':
            self.check_multigoal_simple()
        elif self.experiment_type == 'multigoal_restricted':
            self.check_multigoal_restricted()
    
    def check_single_goal(self):
        """Verifica objetivo único"""
        goal = self.experiment.get('goal', {})
        
        if self.is_goal_reached(goal):
            self.finish_experiment(
                True,
                f"[ÉXITO] Objetivo alcanzado: {self.format_goal(goal)}."
            )
    
    def check_multigoal_simple(self):
        """Verifica objetivos múltiples sin orden"""
        goals = self.experiment.get('goals', [])
        
        for i, goal in enumerate(goals):
            if i not in self.goals_completed and self.is_goal_reached(goal):
                self.goals_completed.append(i)
                message = f"Objetivo {len(self.goals_completed)}/{len(goals)} alcanzado: {self.format_goal(goal)}."
                self.publish_experiment_message(message)
                self.get_logger().info(f'✓ Goal {len(self.goals_completed)}/{len(goals)}: {self.format_goal(goal)}')
        
        # Verificar si todos completados
        if len(self.goals_completed) == len(goals):
            self.finish_experiment(
                True,
                f"[ÉXITO] Todos los objetivos alcanzados ({len(goals)}/{len(goals)})."
            )
    
    def check_multigoal_restricted(self):
        """Verifica objetivos múltiples con restricciones"""
        goals = self.experiment.get('goals', [])
        require_order = self.experiment.get('require_order', False)
        
        # Determinar siguiente objetivo
        if require_order:
            # Debe seguir orden estricto
            next_goal_index = len(self.goals_completed)
            if next_goal_index >= len(goals):
                return  # Ya todos completados
            
            goal = goals[next_goal_index]
            
            if self.is_goal_reached(goal):
                self.goals_completed.append(next_goal_index)
                message = f"Objetivo {len(self.goals_completed)}/{len(goals)} alcanzado (en orden): {self.format_goal(goal)}."
                self.publish_experiment_message(message)
                self.get_logger().info(f'✓ Goal {len(self.goals_completed)}/{len(goals)} (ordered): {self.format_goal(goal)}')
                
                # Verificar si todos completados
                if len(self.goals_completed) == len(goals):
                    self.finish_experiment(
                        True,
                        f"[ÉXITO] Todos los objetivos alcanzados en orden correcto ({len(goals)}/{len(goals)})."
                    )
            else:
                # Verificar si alcanzó objetivo incorrecto (violación de orden)
                for i, other_goal in enumerate(goals):
                    if i != next_goal_index and i not in self.goals_completed:
                        if self.is_goal_reached(other_goal):
                            self.finish_experiment(
                                False,
                                f"[VIOLACIÓN DE ORDEN] Alcanzó objetivo {i+1} antes que objetivo {next_goal_index+1}. Orden requerido violado."
                            )
                            return
        else:
            # Cualquier orden
            for i, goal in enumerate(goals):
                if i not in self.goals_completed and self.is_goal_reached(goal):
                    self.goals_completed.append(i)
                    message = f"Objetivo {len(self.goals_completed)}/{len(goals)} alcanzado: {self.format_goal(goal)}."
                    self.publish_experiment_message(message)
                    self.get_logger().info(f'✓ Goal {len(self.goals_completed)}/{len(goals)}: {self.format_goal(goal)}')
            
            # Verificar si todos completados
            if len(self.goals_completed) == len(goals):
                self.finish_experiment(
                    True,
                    f"[ÉXITO] Todos los objetivos alcanzados ({len(goals)}/{len(goals)})."
                )
    
    def check_forbidden_zones(self):
        """Verifica si el robot está en zonas prohibidas"""
        forbidden_zones = self.experiment.get('forbidden_zones', [])
        forbidden_time_threshold = self.experiment.get('forbidden_time_threshold', 2.0)
        
        if not forbidden_zones:
            return None
        
        current_zone = self.get_current_zone()
        current_time = time.time()
        
        if current_zone in forbidden_zones:
            # Robot está en zona prohibida
            if current_zone not in self.last_zone_entry_time:
                # Primera vez que entra
                self.last_zone_entry_time[current_zone] = current_time
            else:
                # Verificar cuánto tiempo lleva
                time_in_zone = current_time - self.last_zone_entry_time[current_zone]
                if time_in_zone > forbidden_time_threshold:
                    return f"[VIOLACIÓN DE ZONA] Robot permaneció {time_in_zone:.1f}s en zona prohibida '{current_zone}' (límite: {forbidden_time_threshold}s)."
        else:
            # No está en zona prohibida, resetear contador
            if current_zone in self.last_zone_entry_time:
                del self.last_zone_entry_time[current_zone]
        
        return None
    
    # ============= FUNCIONES AUXILIARES =============
    
    def is_goal_reached(self, goal):
        """Verifica si un objetivo ha sido alcanzado"""
        if 'zone' in goal:
            # Objetivo es una zona
            return self.is_in_zone(goal['zone'])
        elif 'coordinates' in goal:
            # Objetivo es una coordenada
            x, y = goal['coordinates']
            return self.is_at_position(x, y, self.goal_threshold)
        return False
    
    def is_in_zone(self, zone_name):
        """Verifica si el robot está en una zona específica"""
        if zone_name not in self.zones or not self.current_pose:
            return False
        
        zone = self.zones[zone_name]
        x = self.current_pose.pose.pose.position.x
        y = self.current_pose.pose.pose.position.y
        
        return (zone['min_x'] <= x <= zone['max_x'] and 
                zone['min_y'] <= y <= zone['max_y'])
    
    def is_at_position(self, target_x, target_y, threshold):
        """Verifica si el robot está en una posición específica"""
        if not self.current_pose:
            return False
        
        x = self.current_pose.pose.pose.position.x
        y = self.current_pose.pose.pose.position.y
        
        distance = math.hypot(target_x - x, target_y - y)
        return distance < threshold
    
    def get_current_zone(self):
        """Obtiene la zona actual del robot (o None)"""
        if not self.current_pose:
            return None
        
        x = self.current_pose.pose.pose.position.x
        y = self.current_pose.pose.pose.position.y
        
        for zone_name, zone in self.zones.items():
            if (zone['min_x'] <= x <= zone['max_x'] and 
                zone['min_y'] <= y <= zone['max_y']):
                return zone_name
        
        return None
    
    def format_goal(self, goal):
        """Formatea un objetivo como texto"""
        if 'zone' in goal:
            return f"zona '{goal['zone']}'"
        elif 'coordinates' in goal:
            x, y = goal['coordinates']
            return f"coordenadas ({x}, {y})"
        return str(goal)
    
    # ============= PUBLICACIÓN Y FINALIZACIÓN =============
    
    def publish_experiment_message(self, message):
        """Publica mensaje de experimento"""
        msg = String()
        msg.data = message
        self.experiment_pub.publish(msg)
    
    def finish_experiment(self, success, message):
        """Finaliza el experimento"""
        self.experiment_running = False
        
        elapsed_time = time.time() - self.start_time
        
        if success:
            self.get_logger().info(f'✅ SUCCESS | {elapsed_time:.1f}s | {message}')
        else:
            self.get_logger().error(f'❌ FAILED | {elapsed_time:.1f}s | {message}')
        
        # Publicar mensaje final
        self.publish_experiment_message(message)
        
        # Esperar un momento para que se publique el mensaje
        time.sleep(1.0)
        
        # Terminar ejecución del nodo
        self.get_logger().info('Shutting down...')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    
    node = ExperimentManagerNode()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()