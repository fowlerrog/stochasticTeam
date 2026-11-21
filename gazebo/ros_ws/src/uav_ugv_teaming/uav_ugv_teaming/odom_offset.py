#!/usr/bin/env python3
"""
odom_offset.py
--------------------
Subscribes to a nav_msgs/msg/Odometry topic, applies a fixed (x, y, z)
position offset and a fixed (roll, pitch, yaw) orientation offset, then
republishes the modified message on a configurable output topic.

Parameters (set via ROS2 parameter server or launch file):
  ~input_topic   (str,   default: "odom")          -- topic to subscribe to
  ~output_topic  (str,   default: "odom_offset")   -- topic to publish on
  ~offset_x      (float, default: 0.0)             -- X position offset (m)
  ~offset_y      (float, default: 0.0)             -- Y position offset (m)
  ~offset_z      (float, default: 0.0)             -- Z position offset (m)
  ~offset_roll   (float, default: 0.0)             -- Roll  offset (rad)
  ~offset_pitch  (float, default: 0.0)             -- Pitch offset (rad)
  ~offset_yaw    (float, default: 0.0)             -- Yaw   offset (rad)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


# ---------------------------------------------------------------------------
# Quaternion helpers (no external dependencies beyond rclpy)
# ---------------------------------------------------------------------------

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """Convert Euler angles (rad) to a quaternion (x, y, z, w)."""
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def quaternion_multiply(q1: tuple, q2: tuple) -> tuple:
    """Multiply two quaternions (x, y, z, w) and return the product."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class OdomOffset(Node):

    def __init__(self):
        super().__init__('odom_offset_node')

        # ── Declare parameters ──────────────────────────────────────────────
        self.declare_parameter('input_topic',  'odom')
        self.declare_parameter('output_topic', 'odom_offset')
        self.declare_parameter('offset_x',     0.0)
        self.declare_parameter('offset_y',     0.0)
        self.declare_parameter('offset_z',     0.0)
        self.declare_parameter('offset_roll',  0.0)
        self.declare_parameter('offset_pitch', 0.0)
        self.declare_parameter('offset_yaw',   0.0)

        # ── Read parameters ─────────────────────────────────────────────────
        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.offset_x     = self.get_parameter('offset_x').value
        self.offset_y     = self.get_parameter('offset_y').value
        self.offset_z     = self.get_parameter('offset_z').value
        self.offset_roll  = self.get_parameter('offset_roll').value
        self.offset_pitch = self.get_parameter('offset_pitch').value
        self.offset_yaw   = self.get_parameter('offset_yaw').value

        # Pre-compute the offset quaternion once
        self._offset_quat = euler_to_quaternion(
            self.offset_roll, self.offset_pitch, self.offset_yaw
        )

        # ── Pub / Sub ────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Odometry, output_topic, 10)
        self._sub = self.create_subscription(
            Odometry, input_topic, self._callback, 10
        )

        self.get_logger().info(
            f"odom_offset started\n"
            f"  Subscribing : {input_topic}\n"
            f"  Publishing  : {output_topic}\n"
            f"  Position    : dx={self.offset_x}  dy={self.offset_y}  dz={self.offset_z}\n"
            f"  Orientation : roll={self.offset_roll}  pitch={self.offset_pitch}  yaw={self.offset_yaw}"
        )

    # ── Callback ─────────────────────────────────────────────────────────────

    def _callback(self, msg: Odometry) -> None:
        out = Odometry()

        # Copy header (keep original stamp; update frame only if needed)
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        # ── Position: simple vector addition ─────────────────────────────────
        out.pose.pose.position.x = msg.pose.pose.position.x + self.offset_x
        out.pose.pose.position.y = msg.pose.pose.position.y + self.offset_y
        out.pose.pose.position.z = msg.pose.pose.position.z + self.offset_z

        # ── Orientation: quaternion multiplication (offset ∘ original) ───────
        orig_q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        new_q = quaternion_multiply(self._offset_quat, orig_q)
        out.pose.pose.orientation = Quaternion(
            x=new_q[0], y=new_q[1], z=new_q[2], w=new_q[3]
        )

        # ── Covariance: pass through unchanged ───────────────────────────────
        out.pose.covariance = msg.pose.covariance

        # ── Twist: pass through unchanged (body-frame velocity unaffected) ───
        out.twist = msg.twist

        self._pub.publish(out)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = OdomOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()