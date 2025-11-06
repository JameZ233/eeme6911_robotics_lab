import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf2_ros import Buffer, TransformListener


def wrap_angle(a):
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

def transform(tf, x, y, z):
    # Robot frame to camera frame
    t = tf.transform.translation
    q = tf.transform.rotation
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    R00 = 1 - 2*(qy*qy + qz*qz); R01 = 2*(qx*qy - qz*qw);   R02 = 2*(qx*qz + qy*qw)
    R10 = 2*(qx*qy + qz*qw);     R11 = 1 - 2*(qx*qx + qz*qz); R12 = 2*(qy*qz - qx*qw)
    R20 = 2*(qx*qz - qy*qw);     R21 = 2*(qy*qz + qx*qw);     R22 = 1 - 2*(qx*qx + qy*qy)
    X = R00*x + R01*y + R02*z + t.x
    Y = R10*x + R11*y + R12*z + t.y
    Z = R20*x + R21*y + R22*z + t.z
    return X, Y, Z

class VisionError(Node):

    def __init__(self):
        super().__init__('vision_error')

        # buffer setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Measurement Setup
        self.d_meas = None
        self.theta_meas = None

        # Subscriptions
        self.create_subscription(Pose2D, '/landmark/position', self.measure_cb, 10)
        self.create_subscription(Odometry, '/odom', self.gt_cb, 10)

        # Publisher
        self.pub_err = self.create_publisher(Pose2D, '/landmark/measurement_error', 10)

        self.get_logger().info(f"Subscription initialized.")

    def measure_cb(self, msg: Pose2D):
        self.d_meas = float(msg.x)
        self.theta_meas = float(msg.theta)

    def gt_cb(self, msg: Odometry):
        if self.d_meas is None or self.theta_meas is None:
            return

        # Transform ground-truth from robot frame to camera frame
        tf = self.tf_buffer.lookup_transform(
            'camera_rgb_optical_frame', 'odom', rclpy.time.Time())

        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        Xc, Yc, Zc = transform(tf, x, y, z)

        # Ground Truth of distance and bearing
        d_true = math.hypot(Xc, Zc)
        theta_true = -math.atan2(Xc, Zc)

        # Errors
        e_d = self.d_meas - d_true
        e_theta = wrap_angle(self.theta_meas - theta_true)

        # Publish error
        err_msg = Pose2D()
        err_msg.x = e_d
        err_msg.y = 0.0
        err_msg.theta = e_theta
        self.pub_err.publish(err_msg)

        self.get_logger().info(
            f"d_meas={self.d_meas:.4f}, theta_meas={self.theta_meas:.4f}, "
            f"d_true={d_true:.4f}, theta_true={theta_true:.4f}, "
            f"e_d={e_d:.4f}, e_theta={e_theta:.4f}"
        )

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    vision_error = VisionError()
    vision_error.spin()
    vision_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
