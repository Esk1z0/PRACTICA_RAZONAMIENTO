#!/usr/bin/env python3
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JsonDumperNode(Node):
    def __init__(self):
        super().__init__("json_dumper")
        self.declare_parameter("topic", "/tu_topic_json")
        self.declare_parameter("out", "/tmp/salida.json")
        self.declare_parameter("mode", "last")  # last | append

        self.topic = self.get_parameter("topic").value
        self.out = Path(self.get_parameter("out").value)
        self.mode = self.get_parameter("mode").value

        self.sub = self.create_subscription(String, self.topic, self.cb, 10)
        self.get_logger().info(f"Escuchando {self.topic} -> {self.out} (mode={self.mode})")

    def cb(self, msg: String):
        s = msg.data.strip()
        try:
            obj = json.loads(s)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON inválido: {e}")
            return

        if self.mode == "append":
            self.out.parent.mkdir(parents=True, exist_ok=True)
            with self.out.open("a", encoding="utf-8") as f:
                f.write(json.dumps(obj, ensure_ascii=False) + "\n")  # JSONL
        else:
            self.out.parent.mkdir(parents=True, exist_ok=True)
            with self.out.open("w", encoding="utf-8") as f:
                json.dump(obj, f, ensure_ascii=False, indent=2)

def main():
    rclpy.init()
    node = JsonDumperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
