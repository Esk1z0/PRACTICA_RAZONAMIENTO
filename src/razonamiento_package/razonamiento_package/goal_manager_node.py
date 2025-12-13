#!/usr/bin/env python3

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped


class GoalManagerNode(Node):
    def __init__(self):
        super().__init__('goal_manager_node')

        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('min_goal_distance', 2.0)
        self.declare_parameter('map_min_x', -4.0)
        self.declare_parameter('map_max_x', 4.0)
        self.declare_parameter('map_min_y', -4.0)
        self.declare_parameter('map_max_y', 4.0)
        self.declare_parameter('map_buffer', 0.8)
        self.declare_parameter('auto_generate', True)
        self.declare_parameter('goal_frame', 'world')
        self.declare_parameter('check_rate_hz', 10.0)
        self.declare_parameter('max_attempts', 200)

        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.map_min_x = float(self.get_parameter('map_min_x').value)
        self.map_max_x = float(self.get_parameter('map_max_x').value)
        self.map_min_y = float(self.get_parameter('map_min_y').value)
        self.map_max_y = float(self.get_parameter('map_max_y').value)
        self.map_buffer = float(self.get_parameter('map_buffer').value)
        self.auto_generate = bool(self.get_parameter('auto_generate').value)
        self.goal_frame = str(self.get_parameter('goal_frame').value)
        self.check_rate_hz = float(self.get_parameter('check_rate_hz').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)

        self.current_pose = None
        self.current_goal = None
        self.goal_reached = False

        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.goal_pub = self.create_publisher(PoseStamped, '/goal', goal_qos)
        self.pose_sub = self.create_subscription(PoseStamped, '/robot/pose', self.pose_callback, pose_qos)
        self.check_timer = self.create_timer(1.0 / self.check_rate_hz, self.check_goal_reached)

    def pose_callback(self, msg: PoseStamped):
        first = self.current_pose is None
        self.current_pose = msg
        if first and self.auto_generate:
            self._publish_new_goal(log_prefix='META INICIAL')

    def check_goal_reached(self):
        if self.current_pose is None or self.current_goal is None:
            return

        dx = self.current_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.pose.position.y
        d = math.hypot(dx, dy)

        if d < self.goal_tolerance and not self.goal_reached:
            self.goal_reached = True
            gx = float(self.current_goal.pose.position.x)
            gy = float(self.current_goal.pose.position.y)
            self.get_logger().info(f'META ALCANZADA: ({gx:.2f}, {gy:.2f}) d={d:.3f}')

            if self.auto_generate:
                self.get_logger().info('REGENERANDO META...')
                self.goal_reached = False
                self._publish_new_goal(log_prefix='NUEVA META')

    def _publish_new_goal(self, log_prefix: str):
        if self.current_pose is None:
            return

        rx = float(self.current_pose.pose.position.x)
        ry = float(self.current_pose.pose.position.y)

        min_x = self.map_min_x + self.map_buffer
        max_x = self.map_max_x - self.map_buffer
        min_y = self.map_min_y + self.map_buffer
        max_y = self.map_max_y - self.map_buffer

        gx = gy = None
        for _ in range(self.max_attempts):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            if math.hypot(x - rx, y - ry) >= self.min_goal_distance:
                gx, gy = x, y
                break

        if gx is None:
            a = random.uniform(-math.pi, math.pi)
            gx = min(max(rx + self.min_goal_distance * math.cos(a), min_x), max_x)
            gy = min(max(ry + self.min_goal_distance * math.sin(a), min_y), max_y)

        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.goal_frame
        m.pose.position.x = float(gx)
        m.pose.position.y = float(gy)
        m.pose.orientation.w = 1.0

        self.current_goal = m
        self.goal_pub.publish(m)
        self.get_logger().info(f'{log_prefix}: ({gx:.2f}, {gy:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
