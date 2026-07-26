import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


def quaternion_conjugate(quaternion):
    return (
        -quaternion.x,
        -quaternion.y,
        -quaternion.z,
        quaternion.w,
    )


def rotate_vector(vector, quaternion):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def relative_position(position, origin_position, origin_orientation):
    displacement = (
        position.x - origin_position.x,
        position.y - origin_position.y,
        position.z - origin_position.z,
    )
    return rotate_vector(displacement, quaternion_conjugate(origin_orientation))


class MetricsRecorder(Node):
    def __init__(self):
        super().__init__("metrics_recorder")
        self.declare_parameter("output_file", "metrics/run.json")
        self.output_file = self.get_parameter("output_file").value
        self.truth_pose = None
        self.truth_origin = None
        self.estimate_origin = None
        self.last_estimate = None
        self.samples = 0
        self.sum_squared_error = 0.0
        self.max_error = 0.0
        self.path_length = 0.0
        self.collisions = 0
        self.create_subscription(
            Odometry, "/simulation/ground_truth/odometry", self._truth, 20
        )
        self.create_subscription(Odometry, "/odometry/filtered", self._estimate, 20)
        self.create_subscription(
            Bool,
            "/simulation/collision",
            lambda message: setattr(
                self, "collisions", self.collisions + int(message.data)
            ),
            10,
        )

    def _truth(self, message):
        self.truth_pose = message.pose.pose
        if self.truth_origin is None:
            self.truth_origin = message.pose.pose

    def _estimate(self, message):
        pose = message.pose.pose
        position = pose.position
        if self.estimate_origin is None:
            self.estimate_origin = pose
        if self.last_estimate is not None:
            self.path_length += math.hypot(
                position.x - self.last_estimate[0],
                position.y - self.last_estimate[1],
            )
        self.last_estimate = (position.x, position.y)
        if (
            self.truth_pose is None
            or self.truth_origin is None
            or self.estimate_origin is None
        ):
            return
        truth = relative_position(
            self.truth_pose.position,
            self.truth_origin.position,
            self.truth_origin.orientation,
        )
        estimate = relative_position(
            position,
            self.estimate_origin.position,
            self.estimate_origin.orientation,
        )
        error = math.sqrt(
            (estimate[0] - truth[0]) ** 2
            + (estimate[1] - truth[1]) ** 2
            + (estimate[2] - truth[2]) ** 2
        )
        self.samples += 1
        self.sum_squared_error += error * error
        self.max_error = max(self.max_error, error)

    def destroy_node(self):
        path = Path(self.output_file).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        ate_rmse = (
            math.sqrt(self.sum_squared_error / self.samples)
            if self.samples
            else None
        )
        path.write_text(
            json.dumps(
                {
                    "samples": self.samples,
                    "alignment": "initial_pose",
                    "ate_rmse_m": ate_rmse,
                    "max_error_m": self.max_error,
                    "estimated_path_length_m": self.path_length,
                    "collisions": self.collisions,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
