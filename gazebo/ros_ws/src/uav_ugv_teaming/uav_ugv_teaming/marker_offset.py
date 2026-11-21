#!/usr/bin/env python3
"""
marker_offset.py
----------------------
Subscribes to a visualization_msgs/msg/Marker topic, applies a fixed (x, y, z)
position offset and a fixed (roll, pitch, yaw) orientation offset to the
marker's pose, then republishes the modified message on a configurable output
topic.

Parameters (set via ROS2 parameter server or launch file):
  ~input_topic   (str,   default: "marker")         -- topic to subscribe to
  ~output_topic  (str,   default: "marker_offset")  -- topic to publish on
  ~offset_x      (float, default: 0.0)              -- X position offset (m)
  ~offset_y      (float, default: 0.0)              -- Y position offset (m)
  ~offset_z      (float, default: 0.0)              -- Z position offset (m)
  ~offset_roll   (float, default: 0.0)              -- Roll  offset (rad)
  ~offset_pitch  (float, default: 0.0)              -- Pitch offset (rad)
  ~offset_yaw    (float, default: 0.0)              -- Yaw   offset (rad)

Notes:
  - All marker fields (type, scale, color, text, mesh, etc.) are passed
    through unchanged — only pose.position and pose.orientation are modified.
  - DELETE / DELETEALL markers are forwarded as-is without touching the pose.
  - A MarkerArray convenience subscriber is also provided via the boolean
    parameter ~array_mode (default: False). When True, the node subscribes to
    visualization_msgs/msg/MarkerArray and publishes a MarkerArray, applying
    the same offset to every marker in the array.
"""

import math
import copy

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion


# ---------------------------------------------------------------------------
# Quaternion helpers (pure Python, no extra dependencies)
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

class MarkerOffset(Node):

    def __init__(self):
        super().__init__('marker_offset')

        # ── Declare parameters ───────────────────────────────────────────────
        self.declare_parameter('input_topic',  'marker')
        self.declare_parameter('output_topic', 'marker_offset')
        self.declare_parameter('array_mode',   False)
        self.declare_parameter('offset_x',     0.0)
        self.declare_parameter('offset_y',     0.0)
        self.declare_parameter('offset_z',     0.0)
        self.declare_parameter('offset_roll',  0.0)
        self.declare_parameter('offset_pitch', 0.0)
        self.declare_parameter('offset_yaw',   0.0)

        # ── Read parameters ──────────────────────────────────────────────────
        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        array_mode   = self.get_parameter('array_mode').value

        self.offset_x     = self.get_parameter('offset_x').value
        self.offset_y     = self.get_parameter('offset_y').value
        self.offset_z     = self.get_parameter('offset_z').value
        self.offset_roll  = self.get_parameter('offset_roll').value
        self.offset_pitch = self.get_parameter('offset_pitch').value
        self.offset_yaw   = self.get_parameter('offset_yaw').value

        # Pre-compute the offset quaternion once at startup
        self._offset_quat = euler_to_quaternion(
            self.offset_roll, self.offset_pitch, self.offset_yaw
        )

        # ── Pub / Sub ────────────────────────────────────────────────────────
        if array_mode:
            self._pub = self.create_publisher(MarkerArray, output_topic, 10)
            self._sub = self.create_subscription(
                MarkerArray, input_topic, self._array_callback, 10
            )
            mode_str = 'MarkerArray'
        else:
            self._pub = self.create_publisher(Marker, output_topic, 10)
            self._sub = self.create_subscription(
                Marker, input_topic, self._marker_callback, 10
            )
            mode_str = 'Marker'

        self.get_logger().info(
            f"marker_offset started\n"
            f"  Mode        : {mode_str}\n"
            f"  Subscribing : {input_topic}\n"
            f"  Publishing  : {output_topic}\n"
            f"  Position    : dx={self.offset_x}  dy={self.offset_y}  dz={self.offset_z}\n"
            f"  Orientation : roll={self.offset_roll}  pitch={self.offset_pitch}  yaw={self.offset_yaw}"
        )

    # ── Pose application ─────────────────────────────────────────────────────

    def _apply_offset(self, marker: Marker) -> Marker:
        """Return a new Marker with the position/orientation offset applied."""
        # DELETE (2) and DELETEALL (3) markers have no meaningful pose — skip.
        if marker.action in (Marker.DELETE, Marker.DELETEALL):
            return marker

        out = copy.copy(marker)  # shallow copy — safe because we reassign pose fields

        # Position: simple vector addition
        out.pose.position.x = marker.pose.position.x + self.offset_x
        out.pose.position.y = marker.pose.position.y + self.offset_y
        out.pose.position.z = marker.pose.position.z + self.offset_z

        # Orientation: quaternion multiplication (offset ∘ original)
        orig_q = (
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
            marker.pose.orientation.w,
        )
        new_q = quaternion_multiply(self._offset_quat, orig_q)
        out.pose.orientation = Quaternion(
            x=new_q[0], y=new_q[1], z=new_q[2], w=new_q[3]
        )

        return out

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _marker_callback(self, msg: Marker) -> None:
        self._pub.publish(self._apply_offset(msg))

    def _array_callback(self, msg: MarkerArray) -> None:
        out = MarkerArray()
        out.markers = [self._apply_offset(m) for m in msg.markers]
        self._pub.publish(out)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MarkerOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()