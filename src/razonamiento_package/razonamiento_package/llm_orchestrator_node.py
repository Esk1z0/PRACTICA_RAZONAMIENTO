#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time


class LLMOrchestratorNode(Node):
    def __init__(self):
        super().__init__('llm_orchestrator_node')
        
        # Parámetros
        self.declare_parameter('mode', 'react')  # 'react' o 'action-effect'
        self.declare_parameter('map_min_x', -10.0)
        self.declare_parameter('map_max_x', 10.0)
        self.declare_parameter('map_min_y', -10.0)
        self.declare_parameter('map_max_y', 10.0)
        
        self.mode = self.get_parameter('mode').value
        self.map_limits = {
            'min_x': float(self.get_parameter('map_min_x').value),
            'max_x': float(self.get_parameter('map_max_x').value),
            'min_y': float(self.get_parameter('map_min_y').value),
            'max_y': float(self.get_parameter('map_max_y').value)
        }
        
        # Estado
        self.tick_counter = 0
        self.current_tick = None
        self.experiment_goal = None
        self.experiment_running = False
        self.world_state = None
        self.current_plan = None
        self.feedback_history = ""
        self.waiting_for_state = False
        self.waiting_for_llm = False
        self.executing_action = False
        self.last_node_sent = None
        self.current_action_index = 0
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.state_request_pub = self.create_publisher(String, '/llm/state_request', 10)
        self.llm_query_pub = self.create_publisher(String, '/llm/query', 10)
        self.event_pub = self.create_publisher(String, '/orchestrator/events', 10)
        
        # Subscribers
        self.create_subscription(String, '/experiment', self.experiment_callback, 10)
        self.create_subscription(String, '/bug2/feedback', self.bug2_callback, 10)
        self.create_subscription(String, '/llm/world_state', self.state_callback, 10)
        self.create_subscription(String, '/llm/response', self.llm_callback, 10)
        
        self.get_logger().info(f'✓ Orchestrator ready | Mode: {self.mode}')
    
    # ===== CALLBACKS =====
    
    def experiment_callback(self, msg: String):
        """Recibe objetivos del experiment_manager"""
        data = msg.data
        
        # Detectar mensajes finales
        if any(tag in data for tag in ['[ÉXITO]', '[TIMEOUT]', '[VIOLACIÓN]', '[MENSAJE_FINAL]']):
            self.publish_event('EXPERIMENT_END', data)
            self.get_logger().info(f'Experiment finished: {data}')
            self.create_timer(10.0, lambda: self.get_logger().info('Shutting down...') or exit(0), once=True)
            return
        
        # Actualizar objetivo
        self.experiment_goal = data
        if not self.experiment_running:
            self.experiment_running = True
            self.start_new_cycle()
    
    def bug2_callback(self, msg: String):
        """Monitorea feedback del Bug2"""
        if not self.executing_action:
            return
        
        feedback = msg.data
        
        if 'REACHED' in feedback:
            self.publish_event('WAYPOINT_REACHED', f'{self.last_node_sent}')
            self.executing_action = False
            self.current_action_index += 1
            self.process_next_action()
        elif 'UNREACHABLE' in feedback:
            self.publish_event('UNREACHABLE', f'{self.last_node_sent} - {feedback}')
            self.executing_action = False
            self.handle_action_failure('unreachable')
    
    def state_callback(self, msg: String):
        """Recibe estado del mundo"""
        if not self.waiting_for_state:
            return
        
        try:
            data = json.loads(msg.data)
            tick = data['meta']['request_id']
            
            if tick != self.current_tick:
                return
            
            self.world_state = data
            self.waiting_for_state = False
            self.publish_event('STATE_RECEIVED', f'{len(data.get("graph", []))} nodos, {len(data.get("frontiers", []))} frontiers')
            self.send_llm_query()
            
        except json.JSONDecodeError:
            self.publish_event('ERROR', 'Invalid world state JSON')
    
    def llm_callback(self, msg: String):
        """Recibe respuesta del LLM"""
        if not self.waiting_for_llm:
            return
        
        try:
            data = json.loads(msg.data)
            
            # El LLM backend envuelve la respuesta en {"id": "...", "response": {...}}
            tick = data.get('id')
            
            if tick != self.current_tick:
                return
            
            response = data.get('response', {})
            self.waiting_for_llm = False
            
            if not self.validate_llm_response(response):
                self.publish_event('VALIDATION_FAILED', 'Invalid LLM response structure')
                self.handle_validation_failure()
                return
            
            self.current_plan = response
            self.publish_event('LLM_RESPONSE', f'{len(response["actions"])} acciones')
            self.execute_plan()
            
        except json.JSONDecodeError:
            self.publish_event('ERROR', 'Invalid LLM response JSON')
        except Exception as e:
            self.publish_event('ERROR', f'Error processing LLM response: {str(e)}')
    
    # ===== CICLO PRINCIPAL =====
    
    def start_new_cycle(self):
        """Inicia un nuevo ciclo de decisión"""
        self.tick_counter += 1
        self.current_tick = f'tick_{self.tick_counter}'
        self.publish_event('CYCLE_START', f'Objetivo: {self.experiment_goal[:50]}...')
        
        # Solicitar estado
        self.waiting_for_state = True
        msg = String()
        msg.data = self.current_tick
        self.state_request_pub.publish(msg)
    
    def send_llm_query(self):
        """Genera y envía prompt al LLM"""
        prompt = self.build_prompt()
        self.waiting_for_llm = True
        
        msg = String()
        msg.data = prompt
        self.llm_query_pub.publish(msg)
    
    def build_prompt(self):
        """Construye el prompt para el LLM"""
        world_state_str = json.dumps(self.world_state) if self.world_state else '{}'
        
        prompt = f"""Objetivo: {self.experiment_goal}

Estado actual: {world_state_str}

Modo: {self.mode}

Feedback del ciclo anterior: {self.feedback_history if self.feedback_history else 'Ninguno (primer ciclo)'}

Tick: {self.current_tick}

IMPORTANTE: 
- Los nodos del grafo tienen "neighbors" que son opciones seguras.
- Los frontiers NO están en el grafo, navegarlos puede ser arriesgado.
- Prefiere ir nodo a nodo usando neighbors para mayor seguridad.
- Cada acción debe tener un fallback (sin sub-fallback).

Genera un plan en JSON estricto:
{{
  "actions": [
    {{
      "mode": "graph_node",
      "target": "cocina_0",
      "fallback": {{"mode": "graph_node", "target": "salon_0"}}
    }}
  ],
  "notes": {{
    "preconditions": "...",
    "expected_result": "..."
  }}
}}

Modos válidos: "graph_node", "frontier", "REPLAN"
"""
        return prompt
    
    # ===== VALIDACIÓN =====
    
    def validate_llm_response(self, response):
        """Valida estructura del JSON del LLM"""
        if 'actions' not in response or 'notes' not in response:
            return False
        
        if not isinstance(response['actions'], list):
            return False
        
        for action in response['actions']:
            if 'mode' not in action:
                return False
            
            mode = action['mode']
            if mode not in ['graph_node', 'frontier', 'REPLAN']:
                return False
            
            if mode != 'REPLAN' and 'target' not in action:
                return False
            
            # Validar fallback
            if 'fallback' in action and action['fallback'] is not None:
                fb = action['fallback']
                if 'mode' not in fb:
                    return False
                if fb['mode'] != 'REPLAN' and 'target' not in fb:
                    return False
                # No permitir fallback recursivo
                if 'fallback' in fb and fb['fallback'] is not None:
                    return False
        
        notes = response['notes']
        if 'preconditions' not in notes or 'expected_result' not in notes:
            return False
        
        return True
    
    def validate_action(self, action):
        """Valida que la acción sea ejecutable"""
        mode = action['mode']
        
        if mode == 'REPLAN':
            return True, None
        
        target = action['target']
        
        if mode == 'graph_node':
            # Verificar que el nodo existe
            graph = self.world_state.get('graph', [])
            if not any(node['id'] == target for node in graph):
                return False, f'Nodo {target} no existe en el grafo'
            
            # Obtener coordenadas
            node = next(n for n in graph if n['id'] == target)
            x, y = node['x'], node['y']
            
        elif mode == 'frontier':
            # Verificar que el frontier existe
            frontiers = self.world_state.get('frontiers', [])
            
            if not any(f.get('id') == target for f in frontiers):
                return False, f'Frontier {target} no existe'
            
            # Obtener coordenadas
            frontier = next(f for f in frontiers if f.get('id') == target)
            x, y = frontier['x'], frontier['y']
        
        # Validar coordenadas dentro del mapa
        if not (self.map_limits['min_x'] <= x <= self.map_limits['max_x']):
            return False, f'X={x} fuera de límites'
        if not (self.map_limits['min_y'] <= y <= self.map_limits['max_y']):
            return False, f'Y={y} fuera de límites'
        
        return True, None
    
    # ===== EJECUCIÓN =====
    
    def execute_plan(self):
        """Ejecuta el plan de acciones"""
        self.current_action_index = 0
        self.process_next_action()
    
    def process_next_action(self):
        """Procesa la siguiente acción del plan"""
        actions = self.current_plan['actions']
        
        if self.current_action_index >= len(actions):
            self.publish_event('CYCLE_END', 'Todas las acciones completadas')
            self.feedback_history = 'Ciclo anterior: Plan completado exitosamente'
            self.start_new_cycle()
            return
        
        action = actions[self.current_action_index]
        
        # Validar acción
        valid, error = self.validate_action(action)
        if not valid:
            self.publish_event('VALIDATION_FAILED', f'Acción {self.current_action_index}: {error}')
            self.use_fallback(action)
            return
        
        # Ejecutar acción
        if action['mode'] == 'REPLAN':
            self.publish_event('REPLAN_REQUESTED', 'LLM solicita replaneación')
            self.feedback_history = f'Ciclo anterior: REPLAN tras {self.current_action_index} acciones'
            self.start_new_cycle()
            return
        
        self.send_goal(action)
    
    def send_goal(self, action):
        """Envía objetivo al Bug2"""
        mode = action['mode']
        target = action['target']
        
        # Obtener coordenadas
        if mode == 'graph_node':
            node = next(n for n in self.world_state['graph'] if n['id'] == target)
            x, y = node['x'], node['y']
        elif mode == 'frontier':
            frontiers = self.world_state['frontiers']
            frontier = next(f for f in frontiers if f.get('id') == target)
            x, y = frontier['x'], frontier['y']
        
        # ELIMINADO: Check de "ya está en el target"
        # Siempre enviar goal y esperar confirmación del Bug2
        
        # Enviar goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = float(x)
        goal_msg.pose.position.y = float(y)
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_msg)
        self.last_node_sent = target
        self.executing_action = True
        self.publish_event('NAVIGATION_START', f'{mode} → {target}')
    
    def use_fallback(self, action):
        """Intenta usar el fallback de la acción"""
        if 'fallback' not in action or action['fallback'] is None:
            self.handle_action_failure('no_fallback')
            return
        
        fallback = action['fallback']
        self.publish_event('USING_FALLBACK', f'{fallback["mode"]} → {fallback.get("target", "REPLAN")}')
        
        # Validar fallback
        valid, error = self.validate_action(fallback)
        if not valid:
            self.publish_event('FALLBACK_FAILED', error)
            self.handle_action_failure('fallback_invalid')
            return
        
        # Ejecutar fallback
        if fallback['mode'] == 'REPLAN':
            self.feedback_history = f'Ciclo anterior: Fallback REPLAN tras fallo en acción {self.current_action_index}'
            self.start_new_cycle()
        else:
            self.send_goal(fallback)
    
    def handle_action_failure(self, reason):
        """Maneja fallo de acción cuando no hay fallback o fallback falla"""
        self.publish_event('ACTION_FAILED', f'Razón: {reason}')
        self.feedback_history = f'Ciclo anterior: Fallo en acción {self.current_action_index} ({reason})'
        self.start_new_cycle()
    
    def handle_validation_failure(self):
        """Maneja fallo de validación del JSON"""
        self.feedback_history = 'Ciclo anterior: Respuesta LLM inválida (estructura JSON incorrecta)'
        self.start_new_cycle()
    
    # ===== UTILIDADES =====
    
    def publish_event(self, event_type, description=''):
        """Publica evento del orchestrator"""
        msg = String()
        msg.data = f'[{event_type}] {self.current_tick} - {description}' if description else f'[{event_type}] {self.current_tick}'
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LLMOrchestratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()