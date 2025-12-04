import rclpy
from rclpy.node import Node

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D, PoseStamped

def q2y(q):
    # Calculation from quaternion to yaw
    sin_cos = 2.0 * (q.w * q.z + q.x * q.y)
    cos_cos = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sin_cos, cos_cos)

def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class EkfError(Node):

    def __init__(self):
        super().__init__('ekf_error')
        
        # Setup last pose
        self.gt_pose = None
        self.ekf_pose = None

        # Subscribers
        self.create_subscription(PoseStamped,
                                 '/tb3/ground_truth/pose', self._gt_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 '/ekf_pose', self._ekf_cb, 10)
        
        # Publishers
        self.err_pose_pub = self.create_publisher(Pose2D, '/ekf_error/pose', 10)

        self.get_logger().info("EKF error node started. Subscribing to /ground_truth and /ekf_pose.")

    def _gt_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = q2y(msg.pose.orientation)
        self.gt_pose = (x, y, theta)
        self._publish_error()

    def _ekf_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = q2y(msg.pose.pose.orientation)
        self.ekf_pose = (x, y, theta)
        self._publish_error()

    def _publish_error(self):
        # Check both are present
        if self.gt_pose is None or self.ekf_pose is None:
            return

        x_gt, y_gt, th_gt = self.gt_pose
        x_ekf, y_ekf, th_ekf = self.ekf_pose

        ex = x_ekf - x_gt
        ey = y_ekf - y_gt
        eth = normalize_angle(th_ekf - th_gt)

        # Error
        err = Pose2D()
        err.x = float(ex)
        err.y = float(ey)
        err.theta = float(eth)
        self.err_pose_pub.publish(err)

        self.get_logger().debug(
            f"Err: ex={ex:.3f} ey={ey:.3f} eθ={math.degrees(eth):.2f}deg | "
        )

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_error = EkfError()
    ekf_error.spin()
    ekf_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
