#!/usr/bin/env python3
# llm_input_bridge_node.py
#
# ROS2 -> JSON de entrada para LLM (incluye mapa como PNG base64 opcional).
#
# Subs:
#   - /robot/pose      [geometry_msgs/PoseStamped]
#   - /goal            [geometry_msgs/PoseStamped]
#   - /robot/sonar_1..16 [sensor_msgs/Range]
#   - /map             [nav_msgs/OccupancyGrid] (opcional)
#
# Pub:
#   - /llm/input_json  [std_msgs/String]
#   - /llm/output_json [std_msgs/String] (mock opcional)

import base64
import json
import math
import time
from typing import Dict, Optional, Any, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class LLMInputBridgeNode(Node):
    def __init__(self):
        super().__init__('llm_input_bridge_node')

        # =========================
        # Parámetros
        # =========================
        self.declare_parameter('publish_rate_hz', 2.0)        # Hz del JSON input
        self.declare_parameter('mock_mode', True)             # publica /llm/output_json simulado
        self.declare_parameter('mock_latency_ms', 0)          # latencia artificial del “LLM” simulado
        self.declare_parameter('max_sonar_range', 5.0)
        self.declare_parameter('robot_frame', 'world')

        # MAPA -> PNG
        self.declare_parameter('include_map', True)           # si False: no incluye nada de /map
        self.declare_parameter('map_as_png', True)            # True: PNG base64 (recomendado)
        self.declare_parameter('map_downsample', 8)           # factor de reducción (8 = muy ligero)
        self.declare_parameter('map_png_max_bytes', 250000)   # límite del base64 (aprox) para no explotar el payload
        self.declare_parameter('map_png_flip_y', True)        # OccupancyGrid suele venir con (0,0) en esquina; flip para “verlo” mejor
        self.declare_parameter('map_png_unknown', 127)        # gris para unknown (-1)
        self.declare_parameter('map_png_free', 255)           # blanco para libre (0)
        self.declare_parameter('map_png_occ', 0)              # negro para ocupado (100)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.mock_latency_ms = int(self.get_parameter('mock_latency_ms').value)
        self.max_sonar_range = float(self.get_parameter('max_sonar_range').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)

        self.include_map = bool(self.get_parameter('include_map').value)
        self.map_as_png = bool(self.get_parameter('map_as_png').value)
        self.map_downsample = int(self.get_parameter('map_downsample').value)
        self.map_png_max_bytes = int(self.get_parameter('map_png_max_bytes').value)
        self.map_png_flip_y = bool(self.get_parameter('map_png_flip_y').value)
        self.map_png_unknown = int(self.get_parameter('map_png_unknown').value)
        self.map_png_free = int(self.get_parameter('map_png_free').value)
        self.map_png_occ = int(self.get_parameter('map_png_occ').value)

        # =========================
        # Estado interno
        # =========================
        self.pose: Optional[PoseStamped] = None
        self.goal: Optional[PoseStamped] = None
        self.sonars: Dict[int, Range] = {}
        self.map_msg: Optional[OccupancyGrid] = None

        # Mapeo ángulos sonares (como vuestro Bug2/minimal)
        self.sensor_angles_deg = {
            1: -90, 2: -50, 3: -30, 4: -10, 5: 10, 6: 30, 7: 50, 8: 90,
            9: 90, 10: 130, 11: 150, 12: 170, 13: -170, 14: -150, 15: -130, 16: -90
        }

        # =========================
        # QoS: goal con TRANSIENT_LOCAL (como goal_manager)
        # =========================
        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # =========================
        # Publishers
        # =========================
        self.input_pub = self.create_publisher(String, '/llm/input_json', 10)
        self.output_pub = self.create_publisher(String, '/llm/output_json', 10)

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(PoseStamped, '/robot/pose', self._pose_cb, 10)
        self.create_subscription(PoseStamped, '/goal', self._goal_cb, goal_qos)

        for i in range(1, 17):
            self.create_subscription(
                Range,
                f'/robot/sonar_{i}',
                lambda msg, sid=i: self._sonar_cb(msg, sid),
                10
            )

        if self.include_map:
            self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)

        # Timer publicación
        self.timer = self.create_timer(1.0 / max(self.publish_rate_hz, 0.1), self._tick)

        self.get_logger().info('LLMInputBridgeNode iniciado')
        self.get_logger().info(f'  publish_rate_hz={self.publish_rate_hz}')
        self.get_logger().info(f'  mock_mode={self.mock_mode} (latency={self.mock_latency_ms}ms)')
        self.get_logger().info(f'  include_map={self.include_map} map_as_png={self.map_as_png} downsample={self.map_downsample}')
        self.get_logger().info('Publica: /llm/input_json y /llm/output_json')

    # =========================
    # Callbacks
    # =========================
    def _pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def _goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def _sonar_cb(self, msg: Range, sensor_id: int):
        self.sonars[sensor_id] = msg

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    # =========================
    # JSON input
    # =========================
    def _build_input_payload(self) -> Optional[Dict[str, Any]]:
        if self.pose is None or self.goal is None or len(self.sonars) < 4:
            return None

        px = float(self.pose.pose.position.x)
        py = float(self.pose.pose.position.y)
        yaw = float(_yaw_from_quat(self.pose.pose.orientation))

        gx = float(self.goal.pose.position.x)
        gy = float(self.goal.pose.position.y)

        dx = gx - px
        dy = gy - py
        dist_goal = float(math.hypot(dx, dy))
        heading_goal = float(_wrap_pi(math.atan2(dy, dx) - yaw))

        sonar_list: List[Dict[str, Any]] = []
        for sid in range(1, 17):
            r = self.sonars.get(sid)
            if r is None:
                rng = self.max_sonar_range
                valid = False
            else:
                rng = float(r.range)
                valid = math.isfinite(rng) and (r.min_range <= rng <= r.max_range)

            sonar_list.append({
                "id": sid,
                "angle_deg": int(self.sensor_angles_deg.get(sid, 0)),
                "range_m": rng,
                "valid": bool(valid)
            })

        payload: Dict[str, Any] = {
            "meta": {
                "schema": "llm_nav_input_v1",
                "timestamp_unix": time.time(),
                "frame": self.robot_frame,
            },
            "robot": {
                "pose": {"x": px, "y": py, "yaw_rad": yaw},
                "goal": {"x": gx, "y": gy},
                "goal_metrics": {
                    "distance_m": dist_goal,
                    "heading_error_rad": heading_goal
                }
            },
            "sensors": {
                "sonars": sonar_list
            },
            "world": {}
        }

        if self.include_map and self.map_msg is not None:
            payload["world"]["map"] = self._map_payload(self.map_msg)

        return payload

    # =========================
    # MAPA -> PNG BASE64
    # =========================
    def _map_payload(self, m: OccupancyGrid) -> Dict[str, Any]:
        w = int(m.info.width)
        h = int(m.info.height)
        res = float(m.info.resolution)
        ox = float(m.info.origin.position.x)
        oy = float(m.info.origin.position.y)

        out: Dict[str, Any] = {
            "schema": "occupancy_grid_snapshot_v2",
            "resolution_m": res,
            "width": w,
            "height": h,
            "origin": {"x": ox, "y": oy},
            "downsample": int(max(1, self.map_downsample)),
        }

        if not self.map_as_png:
            # Si algún día quieres volver a “cells”, lo reintroduces aquí.
            out["note"] = "map_as_png=false: no se incluye imagen"
            return out

        try:
            ds = int(max(1, self.map_downsample))
            grid = np.array(m.data, dtype=np.int16).reshape((h, w))

            # Downsample por “submuestreo” simple (rápido): cada ds
            grid_ds = grid[::ds, ::ds]

            # Render a imagen 8-bit:
            # -1 unknown -> gris
            #  0 free    -> blanco
            # 100 occ    -> negro
            img = np.empty(grid_ds.shape, dtype=np.uint8)

            unknown = (grid_ds < 0)
            free = (grid_ds == 0)
            occ = (grid_ds > 50)  # umbral para ocupado

            img[unknown] = np.uint8(self.map_png_unknown)
            img[free] = np.uint8(self.map_png_free)
            img[occ] = np.uint8(self.map_png_occ)

            # El resto (valores intermedios) los dejamos como “gris medio”
            mid = ~(unknown | free | occ)
            if np.any(mid):
                img[mid] = np.uint8(200)

            if self.map_png_flip_y:
                img = np.flipud(img)

            ok, buf = cv2.imencode('.png', img)
            if not ok:
                raise RuntimeError("cv2.imencode('.png') falló")

            raw = buf.tobytes()
            b64 = base64.b64encode(raw).decode('ascii')

            # Control de tamaño (base64 crece ~33%)
            if len(b64) > self.map_png_max_bytes:
                out["png"] = {
                    "mime": "image/png",
                    "encoding": "base64",
                    "data": None
                }
                out["warning"] = f"PNG base64 truncado por tamaño: {len(b64)} > map_png_max_bytes={self.map_png_max_bytes}"
                out["png_bytes"] = int(len(raw))
                out["png_b64_len"] = int(len(b64))
                return out

            out["png"] = {
                "mime": "image/png",
                "encoding": "base64",
                "data": b64
            }
            out["png_bytes"] = int(len(raw))
            out["png_b64_len"] = int(len(b64))
            out["png_size"] = {"width": int(img.shape[1]), "height": int(img.shape[0])}

            return out

        except Exception as e:
            out["error"] = str(e)
            return out

    # =========================
    # Mock “LLM”
    # =========================
    def _mock_llm_response(self, input_payload: Dict[str, Any]) -> Dict[str, Any]:
        sonars = input_payload["sensors"]["sonars"]

        def _min_range(ids: List[int]) -> float:
            vals = []
            for s in sonars:
                if s["id"] in ids and s["valid"]:
                    vals.append(float(s["range_m"]))
            return min(vals) if vals else self.max_sonar_range

        front = _min_range([4, 5, 3, 6])
        left = _min_range([1, 2, 3, 4])
        right = _min_range([5, 6, 7, 8])

        if front < 0.45:
            action = "right" if right > left else "left"
            strength = min(1.0, (0.6 - front) / 0.6 + 0.2)
        else:
            action = "forward"
            strength = 0.6

        return {
            "meta": {
                "schema": "llm_nav_output_v1",
                "timestamp_unix": time.time(),
                "mock": True
            },
            "decision": {
                "action": action,          # forward | left | right
                "strength": float(max(0.0, min(1.0, strength))),
                "notes": {
                    "front_min_m": float(front),
                    "left_min_m": float(left),
                    "right_min_m": float(right)
                }
            }
        }

    # =========================
    # Tick
    # =========================
    def _tick(self):
        payload = self._build_input_payload()
        if payload is None:
            return

        msg_in = String()
        msg_in.data = json.dumps(payload, ensure_ascii=False)
        self.input_pub.publish(msg_in)

        if self.mock_mode:
            if self.mock_latency_ms > 0:
                time.sleep(self.mock_latency_ms / 1000.0)

            out = self._mock_llm_response(payload)
            msg_out = String()
            msg_out.data = json.dumps(out, ensure_ascii=False)
            self.output_pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = LLMInputBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
