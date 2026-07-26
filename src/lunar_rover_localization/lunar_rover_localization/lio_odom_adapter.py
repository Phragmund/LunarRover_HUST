import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .transforms import compose_transform, inverse_transform


def _tuple_from_pose(pose):
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def _tuple_from_transform(transform):
    return (
        (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ),
        (
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
    )


class LioOdomAdapter(Node):
    def __init__(self):
        super().__init__("lio_odom_adapter")
        self.declare_parameter("input_topic", "/fast_lio/odometry")
        self.declare_parameter("output_topic", "/lio/odometry")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("imu_frame", "livox_imu_frame")
        self.declare_parameter("base_frame", "base_link")

        self.odom_frame = self.get_parameter("odom_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_topic").value, 20
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter("input_topic").value,
            self._callback,
            20,
        )

    def _callback(self, message):
        try:
            base_from_imu = self.tf_buffer.lookup_transform(
                self.base_frame, self.imu_frame, Time()
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Waiting for {self.base_frame} <- {self.imu_frame}: {error}",
                throttle_duration_sec=5.0,
            )
            return

        odom_from_imu = _tuple_from_pose(message.pose.pose)
        base_from_imu_tuple = _tuple_from_transform(base_from_imu.transform)
        odom_from_base = compose_transform(
            odom_from_imu, inverse_transform(base_from_imu_tuple)
        )

        output = Odometry()
        output.header = message.header
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose = message.pose
        translation, rotation = odom_from_base
        output.pose.pose.position.x = translation[0]
        output.pose.pose.position.y = translation[1]
        output.pose.pose.position.z = translation[2]
        output.pose.pose.orientation.x = rotation[0]
        output.pose.pose.orientation.y = rotation[1]
        output.pose.pose.orientation.z = rotation[2]
        output.pose.pose.orientation.w = rotation[3]
        output.twist = message.twist
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

