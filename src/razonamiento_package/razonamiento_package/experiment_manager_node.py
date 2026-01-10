#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
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
        self.declare_parameter('goal_distance_threshold', 0.8)
        self.declare_parameter('check_rate_hz', 5.0)
        
        self.experiment_file = self.get_parameter('experiment_file').value
        self.goal_threshold = float(self.get_parameter('goal_distance_threshold').value)
        self.check_rate = float(self.get_parameter('check_rate_hz').value)
        
        # ============= ESTADO =============
        self.current_pose = None
        self.zones = {}
        self.experiment = None
        self.experiment_type = None
        
        self.goals_completed = []
        self.start_time = None
        self.last_zone_entry_time = {}
        self.experiment_running = False
        
        # ============= QOS =================
        experiment_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        event_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        zone_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ============= PUBLISHERS =============
        self.experiment_pub = self.create_publisher(String, '/experiment', experiment_qos)
        
        # ============= SUBSCRIBERS =============
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            pose_qos
        )
        
        self.zone_sub = self.create_subscription(
            MarkerArray,
            '/zone_markers',
            self.zone_callback,
            zone_qos
        )
        
        # ============= TIMERS =============
        self.check_timer = self.create_timer(1.0 / self.check_rate, self.check_experiment_status)
        self.create_timer(3.0, self.debug_status)
        
        # ============= INICIALIZACIÓN =============
        self.get_logger().info(f'✓ Manager ready | File: {self.experiment_file or "none"}')
        
        if self.experiment_file and self.experiment_file != '':
            self.load_experiment()
        else:
            self.get_logger().warn('No experiment file')
    
    # ============= DEBUG =============
    
    def debug_status(self):
        """Debug cada 3 segundos"""
        if not self.experiment_running:
            self.get_logger().info(
                f'⏳ Waiting | exp:{self.experiment is not None} '
                f'zones:{len(self.zones)} pose:{self.current_pose is not None} '
                f'can_start:{self.can_start_experiment()}'
            )
    
    def can_start_experiment(self) -> bool:
        """Verifica si puede iniciar"""
        if not self.experiment or self.experiment_running or not self.current_pose:
            return False
        if self.experiment_needs_zones() and not self.zones:
            return False
        return True
    
    def experiment_needs_zones(self) -> bool:
        """Verifica si necesita zonas"""
        if not self.experiment:
            return False
        
        if self.experiment_type == 'single_goal':
            goal = self.experiment.get('goal', {})
            return 'zone' in goal
        
        if self.experiment_type in ['multigoal_simple', 'multigoal_restricted']:
            goals = self.experiment.get('goals', [])
            return any('zone' in g for g in goals)
        
        return True
    
    # ============= CALLBACKS =============
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback de pose"""
        first = self.current_pose is None
        self.current_pose = msg
        
        if first:
            x, y = msg.pose.position.x, msg.pose.position.y
            self.get_logger().info(f'✓ Pose: ({x:.1f}, {y:.1f})')
        
        if not self.experiment_running and self.can_start_experiment():
            self.get_logger().info('🚀 Starting experiment')
            self.start_experiment()
    
    def zone_callback(self, msg: MarkerArray):
        """Callback de zonas"""
        zones_before = len(self.zones)
        
        for marker in msg.markers:
            if marker.ns == "zone_labels":
                zone_name = marker.text
                for m in msg.markers:
                    if m.ns == "zone_rectangles" and m.id == marker.id - 2000:
                        cx, cy = m.pose.position.x, m.pose.position.y
                        w, h = m.scale.x, m.scale.y
                        
                        self.zones[zone_name] = {
                            'min_x': cx - w / 2.0,
                            'max_x': cx + w / 2.0,
                            'min_y': cy - h / 2.0,
                            'max_y': cy + h / 2.0
                        }
                        break
        
        if len(self.zones) > zones_before:
            self.get_logger().info(f'✓ Zones: {", ".join(list(self.zones.keys()))}')
            
            if not self.experiment_running and self.can_start_experiment():
                self.get_logger().info('🚀 Starting experiment')
                self.start_experiment()
    
    # ============= EXPERIMENTO =============
    
    def load_experiment(self):
        """Carga experimento desde YAML"""
        try:
            with open(self.experiment_file, 'r') as f:
                config = yaml.safe_load(f)
            
            if 'experiment' not in config:
                self.get_logger().error('Missing "experiment" in YAML')
                return
            
            exp = config['experiment']
            self.experiment_type = exp.get('type')
            self.experiment = exp
            
            if self.experiment_type == 'single_goal':
                goal_str = exp.get('goal', {}).get('zone', str(exp.get('goal', {})))
                self.get_logger().info(f'✓ Loaded: {self.experiment_type} → {goal_str}')
            elif self.experiment_type in ['multigoal_simple', 'multigoal_restricted']:
                num = len(exp.get('goals', []))
                self.get_logger().info(f'✓ Loaded: {self.experiment_type} → {num} goals')
            else:
                self.get_logger().info(f'✓ Loaded: {self.experiment_type}')
            
        except Exception as e:
            self.get_logger().error(f'Error loading: {e}')
    
    def start_experiment(self):
        """Inicia experimento"""
        self.experiment_running = True
        self.start_time = time.time()
        self.goals_completed = []
        
        if self.experiment_type == 'single_goal':
            message = self.generate_single_goal_message()
        elif self.experiment_type == 'multigoal_simple':
            message = self.generate_multigoal_simple_message()
        elif self.experiment_type == 'multigoal_restricted':
            message = self.generate_multigoal_restricted_message()
        else:
            message = f"Unknown: {self.experiment_type}"
        
        self.get_logger().info(f'📤 Publishing: {message[:60]}...')
        self.publish_experiment_message(message)
    
    def generate_single_goal_message(self):
        """Genera mensaje single_goal"""
        goal = self.experiment.get('goal', {})
        desc = self.experiment.get('description', '')
        
        if 'zone' in goal:
            return f"{desc}. Objetivo: navegar a la zona '{goal['zone']}'."
        elif 'coordinates' in goal:
            x, y = goal['coordinates']
            return f"{desc}. Objetivo: navegar a las coordenadas ({x}, {y})."
        else:
            return f"{desc}. Objetivo: {goal}"
    
    def generate_multigoal_simple_message(self):
        """Genera mensaje multigoal_simple"""
        goals = self.experiment.get('goals', [])
        desc = self.experiment.get('description', '')
        
        goal_list = []
        for goal in goals:
            if 'zone' in goal:
                goal_list.append(f"zona '{goal['zone']}'")
            elif 'coordinates' in goal:
                x, y = goal['coordinates']
                goal_list.append(f"coordenadas ({x}, {y})")
        
        goals_str = ', '.join(goal_list)
        return f"{desc}. Objetivos (cualquier orden): {goals_str}."
    
    def generate_multigoal_restricted_message(self):
        """Genera mensaje multigoal_restricted"""
        goals = self.experiment.get('goals', [])
        desc = self.experiment.get('description', '')
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
        
        message = f"{desc}. Objetivos"
        if require_order:
            message += " (ORDEN OBLIGATORIO)"
        message += f": {goals_str}."
        
        if forbidden_zones:
            message += f" RESTRICCIÓN: NO entrar en zonas {forbidden_zones}."
        
        if time_limit:
            message += f" Tiempo límite: {time_limit}s."
        
        return message
    
    # ============= VERIFICACIÓN =============
    
    def check_experiment_status(self):
        """Verifica estado del experimento"""
        if not self.experiment_running or not self.current_pose:
            return
        
        if self.experiment_type == 'multigoal_restricted':
            time_limit = self.experiment.get('time_limit')
            if time_limit:
                elapsed = time.time() - self.start_time
                if elapsed > time_limit:
                    self.finish_experiment(
                        False,
                        f"[TIMEOUT] Tiempo límite de {time_limit}s superado ({elapsed:.1f}s)."
                    )
                    return
        
        if self.experiment_type == 'multigoal_restricted':
            violation = self.check_forbidden_zones()
            if violation:
                self.finish_experiment(False, violation)
                return
        
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
            self.finish_experiment(True, f"[ÉXITO] Objetivo alcanzado: {self.format_goal(goal)}.")
    
    def check_multigoal_simple(self):
        """Verifica objetivos múltiples sin orden"""
        goals = self.experiment.get('goals', [])
        
        for i, goal in enumerate(goals):
            if i not in self.goals_completed and self.is_goal_reached(goal):
                self.goals_completed.append(i)
                message = f"Objetivo {len(self.goals_completed)}/{len(goals)} alcanzado: {self.format_goal(goal)}."
                self.publish_experiment_message(message)
                self.get_logger().info(f'✓ Goal {len(self.goals_completed)}/{len(goals)}')
        
        if len(self.goals_completed) == len(goals):
            self.finish_experiment(True, f"[ÉXITO] Todos los objetivos alcanzados ({len(goals)}/{len(goals)}).")
    
    def check_multigoal_restricted(self):
        """Verifica objetivos múltiples con restricciones"""
        goals = self.experiment.get('goals', [])
        require_order = self.experiment.get('require_order', False)
        
        if require_order:
            next_goal_index = len(self.goals_completed)
            if next_goal_index >= len(goals):
                return
            
            goal = goals[next_goal_index]
            
            if self.is_goal_reached(goal):
                self.goals_completed.append(next_goal_index)
                message = f"Objetivo {len(self.goals_completed)}/{len(goals)} alcanzado (en orden): {self.format_goal(goal)}."
                self.publish_experiment_message(message)
                self.get_logger().info(f'✓ Goal {len(self.goals_completed)}/{len(goals)} (ordered)')
                
                if len(self.goals_completed) == len(goals):
                    self.finish_experiment(True, f"[ÉXITO] Todos los objetivos alcanzados en orden ({len(goals)}/{len(goals)}).")
            else:
                for i, other_goal in enumerate(goals):
                    if i != next_goal_index and i not in self.goals_completed:
                        if self.is_goal_reached(other_goal):
                            self.finish_experiment(
                                False,
                                f"[VIOLACIÓN DE ORDEN] Alcanzó objetivo {i+1} antes que objetivo {next_goal_index+1}."
                            )
                            return
        else:
            for i, goal in enumerate(goals):
                if i not in self.goals_completed and self.is_goal_reached(goal):
                    self.goals_completed.append(i)
                    message = f"Objetivo {len(self.goals_completed)}/{len(goals)} alcanzado: {self.format_goal(goal)}."
                    self.publish_experiment_message(message)
                    self.get_logger().info(f'✓ Goal {len(self.goals_completed)}/{len(goals)}')
            
            if len(self.goals_completed) == len(goals):
                self.finish_experiment(True, f"[ÉXITO] Todos los objetivos alcanzados ({len(goals)}/{len(goals)}).")
    
    def check_forbidden_zones(self):
        """Verifica zonas prohibidas"""
        forbidden_zones = self.experiment.get('forbidden_zones', [])
        forbidden_time_threshold = self.experiment.get('forbidden_time_threshold', 2.0)
        
        if not forbidden_zones:
            return None
        
        current_zone = self.get_current_zone()
        current_time = time.time()
        
        if current_zone in forbidden_zones:
            if current_zone not in self.last_zone_entry_time:
                self.last_zone_entry_time[current_zone] = current_time
            else:
                time_in_zone = current_time - self.last_zone_entry_time[current_zone]
                if time_in_zone > forbidden_time_threshold:
                    return f"[VIOLACIÓN DE ZONA] Robot en zona prohibida '{current_zone}' por {time_in_zone:.1f}s."
        else:
            if current_zone in self.last_zone_entry_time:
                del self.last_zone_entry_time[current_zone]
        
        return None
    
    # ============= AUXILIARES =============
    
    def is_goal_reached(self, goal):
        """Verifica si objetivo alcanzado"""
        if 'zone' in goal:
            return self.is_in_zone(goal['zone'])
        elif 'coordinates' in goal:
            x, y = goal['coordinates']
            return self.is_at_position(x, y, self.goal_threshold)
        return False
    
    def is_in_zone(self, zone_name):
        """Verifica si está en zona"""
        if zone_name not in self.zones or not self.current_pose:
            return False
        
        zone = self.zones[zone_name]
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        return (zone['min_x'] <= x <= zone['max_x'] and 
                zone['min_y'] <= y <= zone['max_y'])
    
    def is_at_position(self, target_x, target_y, threshold):
        """Verifica si está en posición"""
        if not self.current_pose:
            return False
        
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        distance = math.hypot(target_x - x, target_y - y)
        return distance < threshold
    
    def get_current_zone(self):
        """Obtiene zona actual"""
        if not self.current_pose:
            return None
        
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        
        for zone_name, zone in self.zones.items():
            if (zone['min_x'] <= x <= zone['max_x'] and 
                zone['min_y'] <= y <= zone['max_y']):
                return zone_name
        
        return None
    
    def format_goal(self, goal):
        """Formatea objetivo como texto"""
        if 'zone' in goal:
            return f"zona '{goal['zone']}'"
        elif 'coordinates' in goal:
            x, y = goal['coordinates']
            return f"coordenadas ({x}, {y})"
        return str(goal)
    
    # ============= FINALIZACIÓN =============
    
    def publish_experiment_message(self, message):
        """Publica mensaje"""
        msg = String()
        msg.data = message
        self.experiment_pub.publish(msg)
    
    def finish_experiment(self, success, message):
        """Finaliza experimento"""
        self.experiment_running = False
        elapsed_time = time.time() - self.start_time
        
        if success:
            self.get_logger().info(f'✅ SUCCESS | {elapsed_time:.1f}s | {message}')
        else:
            self.get_logger().error(f'❌ FAILED | {elapsed_time:.1f}s | {message}')
        
        self.publish_experiment_message(message)
        time.sleep(1.0)
        
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
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
