#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import Range, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wrap_pi(a: float):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class CoppeliaInterfaceNode(Node):
    def __init__(self):
        super().__init__('coppelia_interface_node')

        self.declare_parameter('robot_name', 'Pioneer_p3dx')
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('max_speed', 2.0)

        self.declare_parameter('scan_frame', 'laser')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)

        self.declare_parameter('angle_min', -120.0 * math.pi / 180.0)
        self.declare_parameter('angle_max', 120.0 * math.pi / 180.0)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 5.0)

        self.declare_parameter('wheel_separation', 0.33)  
        self.declare_parameter('wheel_radius', 0.0975)    


        self.robot_name = self.get_parameter('robot_name').value
        self.update_rate = float(self.get_parameter('update_rate').value)
        self.max_speed = float(self.get_parameter('max_speed').value)

        self.scan_frame = self.get_parameter('scan_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)

        self.client = None
        self.sim = None
        self.robot_handle = None
        self.left_motor = None
        self.right_motor = None
        self.robot_script = None

        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.sonar_pubs = {i: self.create_publisher(Range, f'/robot/sonar_{i}', 10) for i in range(1, 17)}

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.last_t = None

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.connect_to_coppelia()
        self.create_timer(1.0 / self.update_rate, self.update_callback)

    def connect_to_coppelia(self):
        try:
            self.client = RemoteAPIClient()
            self.sim = self.client.getObject('sim')
            self.robot_handle = self.sim.getObject(f'/{self.robot_name}')
            self.left_motor = self.sim.getObject(f'/{self.robot_name}_leftMotor')
            self.right_motor = self.sim.getObject(f'/{self.robot_name}_rightMotor')
            self.robot_script = self.sim.getScript(self.sim.scripttype_childscript, self.robot_handle)
            self.get_logger().info('OK')
            return True
        except Exception as e:
            self.sim = None
            self.get_logger().error(f'FAIL: {e}')
            return False

    def update_callback(self):
        if self.sim is None:
            return
        try:
            self.publish_robot_pose_and_odom()
            self.publish_sonar_readings()
            self.publish_lidar_scan()
        except Exception as e:
            self.get_logger().warn(str(e))

    def publish_robot_pose_and_odom(self):
        pos = self.sim.getObjectPosition(self.robot_handle, -1)
        ori = self.sim.getObjectOrientation(self.robot_handle, -1)

        x = float(pos[0])
        y = float(pos[1])
        z = float(pos[2])
        yaw = float(ori[2])

        now = self.get_clock().now()
        now_msg = now.to_msg()

        qx, qy, qz, qw = yaw_to_quat(yaw)

        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        vx = 0.0
        vy = 0.0
        wz = 0.0

        if self.last_t is not None:
            dt = (now - self.last_t).nanoseconds * 1e-9
            if dt > 1e-6:
                dx = x - self.last_x
                dy = y - self.last_y
                dyaw = wrap_pi(yaw - self.last_yaw)
                vx = dx / dt
                vy = dy / dt
                wz = dyaw / dt

        self.last_x = x
        self.last_y = y
        self.last_yaw = yaw
        self.last_t = now

        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def publish_sonar_readings(self):
        readings = self.sim.callScriptFunction('getAllSonars', self.robot_script)
        now_msg = self.get_clock().now().to_msg()

        for i, d in enumerate(readings, 1):
            if i not in self.sonar_pubs:
                break
            msg = Range()
            msg.header = Header()
            msg.header.stamp = now_msg
            msg.header.frame_id = f'sonar_{i}'
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26
            msg.min_range = 0.0
            msg.max_range = 5.0
            msg.range = float(d)
            self.sonar_pubs[i].publish(msg)

    def publish_lidar_scan(self):
        ranges = self.sim.callScriptFunction('getLaserScan', self.robot_script)
        n = len(ranges)
        if n <= 1:
            return

        now_msg = self.get_clock().now().to_msg()

        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = now_msg
        msg.header.frame_id = self.scan_frame

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = (self.angle_max - self.angle_min) / float(n - 1)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.update_rate
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = [float(x) for x in ranges]

        self.scan_pub.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        if self.sim is None:
            return
        try:
            v = float(msg.linear.x)    # m/s (conceptual)
            w = float(msg.angular.z)   # rad/s (conceptual)

            # Parámetro REAL: separación entre ruedas (metros)
            L = 0.40  # pon aquí tu wheel_separation real

            v_left  = v - w * (L / 2.0)
            v_right = v + w * (L / 2.0)

            # Saturación final
            left  = max(-self.max_speed, min(self.max_speed, v_left))
            right = max(-self.max_speed, min(self.max_speed, v_right))

            self.sim.setJointTargetVelocity(self.left_motor, left)
            self.sim.setJointTargetVelocity(self.right_motor, right)
        except Exception as e:
            self.get_logger().warn(str(e))




def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaInterfaceNode()
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
