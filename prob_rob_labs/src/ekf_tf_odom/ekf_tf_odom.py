import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import TransformStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros


def q2y(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def y2q(yaw: float) -> Quaternion:
    half = yaw / 2.0
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class EkfTfOdom(Node):

    def __init__(self):
        super().__init__('ekf_tf_odom')

        # Parameter setup
        self.last_map_base = None  
        self.last_odom_base = None   

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.map_odom_tf: TransformStamped | None = None

        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/ekf_pose',
            self._ekf_pose_cb,
            10
        )



        self.create_timer(1.0 / 30.0, self._publish_tf)

        self.get_logger().info("map->odom TF publisher started.")

    def _odom_cb(self, msg: Odometry):
        # Find odom pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = q2y(msg.pose.pose.orientation)
        self.last_odom_base = (x, y, yaw)

    def _ekf_pose_cb(self, msg: PoseWithCovarianceStamped):
        # Find ekf_pose
        x_map_base = msg.pose.pose.position.x
        y_map_base = msg.pose.pose.position.y
        yaw_map_base = q2y(msg.pose.pose.orientation)

        self.last_map_base = (x_map_base, y_map_base, yaw_map_base)

        if self.last_odom_base is None:
            # Get odom pose
            self.get_logger().warn("No odom pose yet; cannot compute map->odom transform.")
            return

        x_odom_base, y_odom_base, yaw_odom_base = self.last_odom_base
        yaw_map_odom = normalize_angle(yaw_map_base - yaw_odom_base)

        R_map_odom = np.array([
            [math.cos(yaw_map_odom), -math.sin(yaw_map_odom)],
            [math.sin(yaw_map_odom),  math.cos(yaw_map_odom)],
        ])

        t_map_base = np.array([x_map_base, y_map_base])
        t_odom_base = np.array([x_odom_base, y_odom_base])
        t_map_odom = t_map_base - R_map_odom @ t_odom_base

        # map->odom
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = float(t_map_odom[0])
        tf_msg.transform.translation.y = float(t_map_odom[1])
        tf_msg.transform.translation.z = 0.0

        q = y2q(yaw_map_odom)
        tf_msg.transform.rotation = q

        self.map_odom_tf = tf_msg

    def _publish_tf(self):
        if self.map_odom_tf is None:
            return

        tf_msg = self.map_odom_tf
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(tf_msg)

    def spin(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    node = EkfTfOdom()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
