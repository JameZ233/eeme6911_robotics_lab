import rclpy
from rclpy.node import Node

import math
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros

def q2y(q):
    # Quaternion to yaw 
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def y2q(yaw):
    # Yaw to quaternion
    half = yaw / 2.0
    q = type('Q', (), {})()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q

def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class EkfTfOdom(Node):

    def __init__(self):
        super().__init__('ekf_tf_odom')
        # TF buffer to read odom
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TF map -> odom
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.map_odom_tf: TransformStamped | None = None

        # Subscribe
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/ekf_pose',
            self._ekf_pose_cb,
            10
        )

        # Publish at 30 Hz
        self.create_timer(1.0 / 30.0, self._publish_tf)

        self.get_logger().info("Odom TF publisher started.")

    def _ekf_pose_cb(self, msg: PoseWithCovarianceStamped):
        # Pose of base_link in map frame
        x_map_base = msg.pose.pose.position.x
        y_map_base = msg.pose.pose.position.y
        yaw_map_base = q2y(msg.pose.pose.orientation)

        # odom -> base_link transform
        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Could not get odom->base_link TF: {e}")
            return

        x_odom_base = tf_odom_base.transform.translation.x
        y_odom_base = tf_odom_base.transform.translation.y
        yaw_odom_base = q2y(tf_odom_base.transform.rotation)

        # map->odom
        yaw_map_odom = normalize_angle(yaw_map_base - yaw_odom_base)
        R_map_odom = np.array([
            [math.cos(yaw_map_odom), -math.sin(yaw_map_odom)],
            [math.sin(yaw_map_odom),  math.cos(yaw_map_odom)],
        ])
        t_map_base = np.array([x_map_base, y_map_base])
        t_odom_base = np.array([x_odom_base, y_odom_base])

        t_map_odom = t_map_base - R_map_odom @ t_odom_base

        # Build TF message
        tf_msg = TransformStamped()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = float(t_map_odom[0])
        tf_msg.transform.translation.y = float(t_map_odom[1])
        tf_msg.transform.translation.z = 0.0

        q = y2q(yaw_map_odom)
        tf_msg.transform.rotation.x = q.x
        tf_msg.transform.rotation.y = q.y
        tf_msg.transform.rotation.z = q.z
        tf_msg.transform.rotation.w = q.w

        self.map_odom_tf = tf_msg

    def _publish_tf(self):
        if self.map_odom_tf is None:
            return

        tf_msg = self.map_odom_tf
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(tf_msg)

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_tf_odom = EkfTfOdom()
    ekf_tf_odom.spin()
    ekf_tf_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
