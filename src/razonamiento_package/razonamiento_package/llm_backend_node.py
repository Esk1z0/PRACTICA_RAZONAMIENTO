#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


class LLMBackendNode(Node):
    def __init__(self):
        super().__init__('llm_backend_node')
        
        # ============= PARÁMETROS =============
        self.declare_parameter('llm_base_url', 'http://localhost:1234/v1')
        self.declare_parameter('llm_api_key', 'lm-studio')
        self.declare_parameter('llm_model', 'meta-llama-3.1-8b-instruct')
        self.declare_parameter('llm_temperature', 0.2)
        self.declare_parameter('llm_max_tokens', 500)
        self.declare_parameter('system_prompt', 
            '''Eres un robot navegador. Debes responder SIEMPRE en JSON estricto con esta estructura:

{
  "actions": [
    {
      "mode": "graph_node",
      "target": "nombre_del_nodo",
      "fallback": {
        "mode": "graph_node",
        "target": "nodo_alternativo"
      }
    }
  ],
  "notes": {
    "preconditions": "Condiciones que te llevaron a este plan",
    "expected_result": "Resultado esperado del plan"
  }
}

Modos válidos: "graph_node" (nodo del grafo), "frontier" (área sin explorar), "REPLAN" (pedir nuevo estado).
- "graph_node" y "frontier" requieren "target" con el ID del nodo/frontier.
- "REPLAN" NO tiene "target".
- Cada acción DEBE tener un "fallback" (plan B), que sigue la misma estructura pero SIN sub-fallback.
- El fallback puede ser null solo si no hay alternativa posible.

NO añadas texto fuera del JSON. Responde SOLO con el JSON válido.''')
        self.declare_parameter('enable_debug_logs', False)
        
        # Obtener parámetros
        self.base_url = self.get_parameter('llm_base_url').value
        self.api_key = self.get_parameter('llm_api_key').value
        self.model = self.get_parameter('llm_model').value
        self.temperature = float(self.get_parameter('llm_temperature').value)
        self.max_tokens = int(self.get_parameter('llm_max_tokens').value)
        self.system_prompt = self.get_parameter('system_prompt').value
        self.debug_logs = bool(self.get_parameter('enable_debug_logs').value)
        
        # ============= VERIFICAR DISPONIBILIDAD =============
        if not OPENAI_AVAILABLE:
            self.get_logger().error('=' * 60)
            self.get_logger().error('ERROR: openai library not installed!')
            self.get_logger().error('Install with: pip install openai')
            self.get_logger().error('=' * 60)
            self.client = None
        else:
            try:
                self.client = OpenAI(
                    base_url=self.base_url,
                    api_key=self.api_key
                )
                self.get_logger().info('✓ OpenAI client initialized successfully')
                
                # Test de conexión (opcional)
                self.test_connection()
                
            except Exception as e:
                self.get_logger().error(f'ERROR initializing OpenAI client: {e}')
                self.client = None
        
        # ============= COMUNICACIÓN ROS2 (Request-Response Pattern) =============
        # Subscriber: recibe queries
        self.request_sub = self.create_subscription(
            String,
            '/llm/query',
            self.query_callback,
            10
        )
        
        # Publisher: publica respuestas
        self.response_pub = self.create_publisher(
            String,
            '/llm/response',
            10
        )
        
        # ============= ESTADÍSTICAS =============
        self.total_queries = 0
        self.successful_queries = 0
        self.failed_queries = 0
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('LLM Backend Node Started')
        self.get_logger().info(f'Base URL: {self.base_url}')
        self.get_logger().info(f'Model: {self.model}')
        self.get_logger().info(f'Temperature: {self.temperature}')
        self.get_logger().info(f'Max tokens: {self.max_tokens}')
        self.get_logger().info(f'Debug logs: {self.debug_logs}')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Listening on: /llm/query (std_msgs/String)')
        self.get_logger().info('Publishing to: /llm/response (std_msgs/String)')
        self.get_logger().info('=' * 60)
    
    def test_connection(self):
        """Test opcional de conexión al servidor LLM"""
        try:
            self.get_logger().info('Testing connection to LLM server...')
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "user", "content": "Responde solo con: OK"}
                ],
                temperature=0.0,
                max_tokens=10
            )
            answer = response.choices[0].message.content
            self.get_logger().info(f'✓ Connection test successful: "{answer}"')
        except Exception as e:
            self.get_logger().warn(f'Connection test failed: {e}')
            self.get_logger().warn('LLM server may not be running')
    
    def query_callback(self, msg):
        """Callback cuando llega una consulta al LLM"""
        state_text = msg.data
        self.total_queries += 1
        
        # Estimar tokens de entrada (aproximación: 1 token ≈ 4 chars)
        input_tokens = len(state_text) // 4
        
        # Llamar al LLM
        response_text, output_tokens = self.decide(state_text)
        
        if response_text is not None:
            self.successful_queries += 1
            
            # Publicar respuesta
            response_msg = String()
            response_msg.data = response_text
            self.response_pub.publish(response_msg)
            
            self.get_logger().info(
                f'Query #{self.total_queries} ✓ | '
                f'In: ~{input_tokens}t | Out: ~{output_tokens}t'
            )
        else:
            self.failed_queries += 1
            
            # Publicar respuesta de error
            error_response = json.dumps({
                "action": "error",
                "args": {},
                "confidence": 0.0,
                "error": "LLM query failed"
            })
            response_msg = String()
            response_msg.data = error_response
            self.response_pub.publish(response_msg)
            
            self.get_logger().error(f'Query #{self.total_queries} ✗')
    
    def decide(self, state_text: str):
        """
        Llama al LLM con el state_text y devuelve la respuesta y tokens.
        Retorna (None, 0) si hay error.
        """
        if self.client is None:
            self.get_logger().error('OpenAI client not initialized - cannot process query')
            return None, 0
        
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": state_text}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )
            
            # Extraer respuesta
            answer = response.choices[0].message.content
            
            # Obtener tokens (si disponible)
            output_tokens = 0
            if hasattr(response, 'usage') and response.usage:
                output_tokens = response.usage.completion_tokens
            else:
                # Estimación si no está disponible
                output_tokens = len(answer) // 4
            
            if self.debug_logs:
                self.get_logger().debug(f'LLM response received: {len(answer)} chars')
                self.get_logger().debug(f'Finish reason: {response.choices[0].finish_reason}')
                if hasattr(response, 'usage'):
                    self.get_logger().debug(f'Tokens - Total: {response.usage.total_tokens}, Prompt: {response.usage.prompt_tokens}, Completion: {response.usage.completion_tokens}')
            
            return answer, output_tokens
        
        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {e}')
            if self.debug_logs:
                import traceback
                self.get_logger().error(traceback.format_exc())
            return None, 0


def main(args=None):
    rclpy.init(args=args)
    
    node = LLMBackendNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM Backend Node...')
    finally:
        node.get_logger().info('=' * 60)
        node.get_logger().info('Final Statistics:')
        node.get_logger().info(f'  Total queries: {node.total_queries}')
        node.get_logger().info(f'  Successful: {node.successful_queries}')
        node.get_logger().info(f'  Failed: {node.failed_queries}')
        if node.total_queries > 0:
            success_rate = (node.successful_queries / node.total_queries) * 100
            node.get_logger().info(f'  Success rate: {success_rate:.1f}%')
        node.get_logger().info('=' * 60)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()