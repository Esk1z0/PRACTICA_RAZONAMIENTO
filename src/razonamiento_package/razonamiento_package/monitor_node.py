#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import json
import os
from datetime import datetime


class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # ============= PARÁMETROS =============
        self.declare_parameter('output_dir', './logs')
        self.declare_parameter('log_filename', '')
        self.declare_parameter('buffer_size', 100)
        self.declare_parameter('pretty_json', True)  # NUEVO: formato bonito
        
        self.output_dir = self.get_parameter('output_dir').value
        self.log_filename = self.get_parameter('log_filename').value
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.pretty_json = self.get_parameter('pretty_json').value
        
        # ============= ESTADO =============
        self.tick_counter = 0
        self.log_buffer = []
        self.log_file = None
        self.start_time = None
        
        # ============= INICIALIZAR ARCHIVO =============
        self.initialize_log_file()
        
        # ============ QOS ================
        event_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # ============= SUBSCRIBERS =============
        self.create_subscription(
            String, '/orchestrator/events',
            lambda msg: self.log_message('ORCHESTRATOR_EVENT', msg.data),
            event_qos
        )
        
        self.create_subscription(
            String, '/experiment',
            lambda msg: self.log_message('EXPERIMENT', msg.data),
            10
        )
        
        self.create_subscription(
            String, '/bug2/feedback',
            lambda msg: self.log_message('BUG2_FEEDBACK', msg.data),
            10
        )
        
        self.create_subscription(
            String, '/llm/state_request',
            lambda msg: self.log_message('STATE_REQUEST', msg.data),
            10
        )
        
        self.create_subscription(
            String, '/llm/world_state',
            lambda msg: self.log_message('WORLD_STATE', msg.data),
            10
        )
        
        self.create_subscription(
            String, '/llm/query',
            lambda msg: self.log_message('LLM_QUERY', msg.data),
            10
        )
        
        self.create_subscription(
            String, '/llm/response',
            lambda msg: self.log_message('LLM_RESPONSE', msg.data),
            10
        )
        
        self.create_subscription(
            PoseStamped, '/goal',
            self.goal_callback,
            10
        )
        
        # ============= TIMER PARA FLUSH =============
        self.flush_timer = self.create_timer(5.0, self.flush_buffer)
        
        self.get_logger().info(f'✓ Monitor ready | Output: {self.log_file.name}')
    
    def initialize_log_file(self):
        """Inicializa el archivo de log"""
        os.makedirs(self.output_dir, exist_ok=True)
        
        if not self.log_filename or self.log_filename == '':
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.log_filename = f'monitor_{timestamp}.jsonl'
        
        log_path = os.path.join(self.output_dir, self.log_filename)
        self.log_file = open(log_path, 'a')
        
        self.start_time = self.get_clock().now()
        header = {
            'type': 'HEADER',
            'timestamp': self.start_time.nanoseconds / 1e9,
            'datetime': datetime.now().isoformat(),
            'node': 'monitor_node'
        }
        
        if self.pretty_json:
            self.log_file.write(json.dumps(header, indent=2) + '\n' + ('='*80) + '\n')
        else:
            self.log_file.write(json.dumps(header) + '\n')
        self.log_file.flush()
    
    def log_message(self, topic_type, data):
        """Registra un mensaje en el log"""
        self.tick_counter += 1
        
        current_time = self.get_clock().now()
        elapsed = (current_time.nanoseconds - self.start_time.nanoseconds) / 1e9
        
        # ============= PARSEAR DATA SI ES JSON =============
        parsed_data = self.try_parse_json(data)
        
        log_entry = {
            'tick': self.tick_counter,
            'timestamp': round(current_time.nanoseconds / 1e9, 3),
            'elapsed': round(elapsed, 3),
            'type': topic_type,
            'data': parsed_data  # ← Ahora puede ser dict o string
        }
        
        # ============= FORMATEAR SEGÚN PREFERENCIA =============
        if self.pretty_json:
            formatted = json.dumps(log_entry, indent=2, ensure_ascii=False)
            self.log_buffer.append(formatted + '\n' + ('-'*80))
        else:
            self.log_buffer.append(json.dumps(log_entry, ensure_ascii=False))
        
        # Flush si el buffer está lleno
        if len(self.log_buffer) >= self.buffer_size:
            self.flush_buffer()
    
    def try_parse_json(self, data: str):
        """Intenta parsear data como JSON, si falla retorna el string original"""
        try:
            return json.loads(data)
        except (json.JSONDecodeError, TypeError):
            return data
    
    def goal_callback(self, msg: PoseStamped):
        """Callback para goals"""
        goal_data = {
            'x': round(msg.pose.position.x, 2),
            'y': round(msg.pose.position.y, 2),
            'frame': msg.header.frame_id
        }
        self.log_message('GOAL_SENT', json.dumps(goal_data))
    
    def flush_buffer(self):
        """Escribe el buffer al archivo"""
        if self.log_buffer:
            for entry in self.log_buffer:
                self.log_file.write(entry + '\n')
            self.log_file.flush()
            self.log_buffer.clear()
    
    def shutdown(self):
        """Cierra el archivo de log"""
        if self.log_file and not self.log_file.closed:
            self.flush_buffer()
            
            end_time = self.get_clock().now()
            total_elapsed = (end_time.nanoseconds - self.start_time.nanoseconds) / 1e9
            
            footer = {
                'type': 'FOOTER',
                'timestamp': round(end_time.nanoseconds / 1e9, 3),
                'datetime': datetime.now().isoformat(),
                'total_ticks': self.tick_counter,
                'total_time': round(total_elapsed, 3)
            }
            
            if self.pretty_json:
                self.log_file.write('\n' + ('='*80) + '\n')
                self.log_file.write(json.dumps(footer, indent=2) + '\n')
            else:
                self.log_file.write(json.dumps(footer) + '\n')
            
            self.log_file.flush()
            self.log_file.close()
            
            self.get_logger().info(f'✓ Log closed: {self.tick_counter} ticks, {total_elapsed:.1f}s')


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()