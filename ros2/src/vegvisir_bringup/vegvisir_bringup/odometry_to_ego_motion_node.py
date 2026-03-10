# Copyright (c) Sensrad 2025-2026
"""Adapter node: converts nav_msgs/Odometry to oden_interfaces/EgoMotion.

KISS-ICP publishes nav_msgs/Odometry, but VegvisirNode subscribes to
oden_interfaces/EgoMotion via ApproximateTime sync.  This node bridges the
two by converting absolute pose + computing per-message deltas.
"""

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from oden_interfaces.msg import EgoMotion


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 4x4 homogeneous transformation matrix."""
    T = np.eye(4)
    T[0, 0] = 1.0 - 2.0 * (y * y + z * z)
    T[0, 1] = 2.0 * (x * y - z * w)
    T[0, 2] = 2.0 * (x * z + y * w)
    T[1, 0] = 2.0 * (x * y + z * w)
    T[1, 1] = 1.0 - 2.0 * (x * x + z * z)
    T[1, 2] = 2.0 * (y * z - x * w)
    T[2, 0] = 2.0 * (x * z - y * w)
    T[2, 1] = 2.0 * (y * z + x * w)
    T[2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return T


def _matrix_to_quat(T: np.ndarray) -> tuple[float, float, float, float]:
    """Extract quaternion (x, y, z, w) from the rotation part of a 4x4 matrix."""
    R = T[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class OdometryToEgoMotionNode(Node):
    """Converts nav_msgs/Odometry to oden_interfaces/EgoMotion with delta computation."""

    def __init__(self) -> None:
        super().__init__("odometry_to_ego_motion")
        self._sub = self.create_subscription(
            Odometry, "odometry_in", self._odometry_callback, 10
        )
        self._pub = self.create_publisher(EgoMotion, "ego_motion_out", 10)
        self._prev_T: np.ndarray | None = None

    def _odometry_callback(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        T_curr = _quat_to_matrix(ori.x, ori.y, ori.z, ori.w)
        T_curr[0, 3] = pos.x
        T_curr[1, 3] = pos.y
        T_curr[2, 3] = pos.z

        if self._prev_T is not None:
            T_delta = np.linalg.inv(self._prev_T) @ T_curr
        else:
            T_delta = np.eye(4)

        self._prev_T = T_curr.copy()

        dx, dy, dz, dw = _matrix_to_quat(T_delta)

        ego = EgoMotion()
        ego.header = msg.header

        ego.odometry_is_valid = True
        ego.velocity_is_valid = False
        ego.rotational_rate_is_valid = False

        # Absolute pose
        ego.translation_x = float(pos.x)
        ego.translation_y = float(pos.y)
        ego.translation_z = float(pos.z)
        ego.rotation_quat_x = float(ori.x)
        ego.rotation_quat_y = float(ori.y)
        ego.rotation_quat_z = float(ori.z)
        ego.rotation_quat_w = float(ori.w)

        # Delta pose
        ego.delta_translation_x = float(T_delta[0, 3])
        ego.delta_translation_y = float(T_delta[1, 3])
        ego.delta_translation_z = float(T_delta[2, 3])
        ego.delta_rotation_quat_x = float(dx)
        ego.delta_rotation_quat_y = float(dy)
        ego.delta_rotation_quat_z = float(dz)
        ego.delta_rotation_quat_w = float(dw)

        # Velocity fields not used by VegvisirNode
        ego.velocity_x = 0.0
        ego.velocity_y = 0.0
        ego.velocity_z = 0.0
        ego.rotational_rate_x = 0.0
        ego.rotational_rate_y = 0.0
        ego.rotational_rate_z = 0.0

        self._pub.publish(ego)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdometryToEgoMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
