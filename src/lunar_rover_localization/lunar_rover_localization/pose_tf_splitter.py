import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .transforms import (
    quat_conjugate,
    quat_multiply,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


class PoseTfSplitter(Node):
    def __init__(self):
        super().__init__("pose_tf_splitter")
        self.declare_parameter("input_topic", "/odometry/filtered")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("footprint_frame", "base_footprint")
        self.declare_parameter("base_frame", "base_link")
        self.odom_frame = self.get_parameter("odom_frame").value
        self.footprint_frame = self.get_parameter("footprint_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter("input_topic").value,
            self._callback,
            20,
        )

    def _callback(self, message):
        orientation = message.pose.pose.orientation
        full_q = (orientation.x, orientation.y, orientation.z, orientation.w)
        yaw_q = quaternion_from_yaw(yaw_from_quaternion(full_q))
        residual_q = quat_multiply(quat_conjugate(yaw_q), full_q)

        odom_to_footprint = TransformStamped()
        odom_to_footprint.header.stamp = message.header.stamp
        odom_to_footprint.header.frame_id = self.odom_frame
        odom_to_footprint.child_frame_id = self.footprint_frame
        odom_to_footprint.transform.translation.x = message.pose.pose.position.x
        odom_to_footprint.transform.translation.y = message.pose.pose.position.y
        odom_to_footprint.transform.translation.z = 0.0
        odom_to_footprint.transform.rotation.x = yaw_q[0]
        odom_to_footprint.transform.rotation.y = yaw_q[1]
        odom_to_footprint.transform.rotation.z = yaw_q[2]
        odom_to_footprint.transform.rotation.w = yaw_q[3]

        footprint_to_base = TransformStamped()
        footprint_to_base.header.stamp = message.header.stamp
        footprint_to_base.header.frame_id = self.footprint_frame
        footprint_to_base.child_frame_id = self.base_frame
        footprint_to_base.transform.translation.z = message.pose.pose.position.z
        footprint_to_base.transform.rotation.x = residual_q[0]
        footprint_to_base.transform.rotation.y = residual_q[1]
        footprint_to_base.transform.rotation.z = residual_q[2]
        footprint_to_base.transform.rotation.w = residual_q[3]

        self.broadcaster.sendTransform([odom_to_footprint, footprint_to_base])


def main(args=None):
    rclpy.init(args=args)
    node = PoseTfSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

