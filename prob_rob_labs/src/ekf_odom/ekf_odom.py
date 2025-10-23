#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer

class EkfOdom(Node):
    def __init__(self):
        super().__init__('ekf_odom')
        self.get_logger().info("Initialized EKF Odom")
        self.create_subscription(Twist, '/cmd_vel', self.controlCallback, 10)

        # Time-synced inputs
        self.pose_sub  = Subscriber(self, PoseStamped,  "/tb3/ground_truth/pose")
        self.twist_sub = Subscriber(self, TwistStamped, "/tb3/ground_truth/twist")
        self.imu_sub   = Subscriber(self, Imu,          "/imu")

        self.time_sync = ApproximateTimeSynchronizer(
            [self.pose_sub, self.twist_sub, self.imu_sub], queue_size=10, slop=0.05
        )
        self.time_sync.registerCallback(self.SyncCallback)

        # Geometry and parameters defined
        self.r_w = 0.033
        self.R = 0.1435 / 2
        self.tau_v = 1.772
        self.tau_w = 0.080
        self.Gv = 2.0
        self.Gw = 0.5

        # EKF state setup
        self.x = np.zeros(5)           
        self.P = np.eye(5)
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-3, 1e-3])
        self.R_mea = np.diag([0.05**2, 0.05**2, 0.02**2, 0.02**2, 0.02**2])

        # Sticky control
        self.u_v = 0.0
        self.u_w = 0.0

        # TIme setup
        self.last_time = None
        self.last_v = 0.0

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/ekf_odom', 10)

        # C Matrix
        self.C = np.array([
            [0.0, 0.0, 0.0, 1.0/self.r_w,  self.R/self.r_w],
            [0.0, 0.0, 0.0, 1.0/self.r_w, -self.R/self.r_w],
            [0.0, 0.0, 0.0, 0.0,           1.0           ]
        ])


    def controlCallback(self, msg: Twist):
        self.u_v = msg.linear.x
        self.u_w = msg.angular.z

    def SyncCallback(self, pose_msg, twist_msg, imu_msg: Imu):
        # Read from IMU and calculate theta
        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w
        t_now = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        theta_m = math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0 * (qy*qy + qz*qz))
        w_m = imu_msg.angular_velocity.z

        # Read Acceleration
        a_x = imu_msg.linear_acceleration.x 


        if self.last_time is None:
            self.last_time = t_now
            self.last_v = 0.0
            self.x = np.array([0.0, 0.0, theta_m, 0.0, w_m], dtype=float)
            return

        if t_now <= self.last_time:
            return  # drop out-of-order stamps in case

        dt = max(1e-4, t_now - self.last_time)
        self.last_time = t_now

        # Calculate velocity x
        v_m = self.last_v + a_x * dt
        self.last_v = v_m

        # Build vector z 
        z = np.array([self.x[0], self.x[1], theta_m, v_m, w_m], dtype=float)

        # EKF step
        self.predict(dt)
        self.update(z)
        self.publish_odom(t_now)

    def _wrap(self, ang):
        return math.atan2(math.sin(ang), math.cos(ang))

    def B_matrix(self, av, aw):
        return np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [(1.0 - av) * self.Gv, 0.0],
            [0.0, (1.0 - aw) * self.Gw]
        ])

    def predict(self, dt: float):
        X, Y, theta, v, w = self.x

        # forgetting factors
        av = 0.997404 
        aw = 0.943942

        # state propagation
        X1 = X + dt * v * math.cos(theta)
        Y1 = Y + dt * v * math.sin(theta)
        theta1 = theta + dt * w
        v1 = av * v + (1.0 - av) * self.Gv * self.u_v
        w1 = aw * w + (1.0 - aw) * self.Gw * self.u_w
        self.x = np.array([X1, Y1, theta1, v1, w1])

        # Jacobian F
        F = np.array([
            [1.0, 0.0, -dt * v * math.sin(theta),  dt * math.cos(theta), 0.0],
            [0.0, 1.0,  dt * v * math.cos(theta),  dt * math.sin(theta), 0.0],
            [0.0, 0.0, 1.0,                         0.0,                 dt ],
            [0.0, 0.0, 0.0,                         av,                  0.0],
            [0.0, 0.0, 0.0,                         0.0,                 aw ]
        ])

        B = self.B_matrix(av, aw)

        # process noise
        Q = self.Q * dt
        self.P = F @ self.P @ F.T + Q

    def update(self, z: np.ndarray):
        H = np.eye(5)
        y = z - self.x
        y[2] = self._wrap(y[2])
        S = self.P + self.R_mea
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(5) - K) @ self.P
        self.get_logger().info(f"x = {self.x}")

    def publish_odom(self, t: float):
        X, Y, theta, v, w = self.x
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t - int(t)) * 1e9)

        msg.pose.pose.position.x = float(X)
        msg.pose.pose.position.y = float(Y)
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)

        # map P into odom covariances
        P = self.P
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36
        pose_cov[0]  = float(P[0, 0])
        pose_cov[7]  = float(P[1, 1])
        pose_cov[35] = float(P[2, 2])
        pose_cov[1]  = float(P[0, 1]); pose_cov[6]  = float(P[1, 0])
        pose_cov[5]  = float(P[0, 2]); pose_cov[30] = float(P[2, 0])
        pose_cov[11] = float(P[1, 2]); pose_cov[31] = float(P[2, 1])

        twist_cov[0]  = float(P[3, 3])
        twist_cov[35] = float(P[4, 4])
        twist_cov[5]  = float(P[3, 4]); twist_cov[30] = float(P[4, 3])

        self.get_logger().info(f"Covariance Matrix of Pose is here {pose_cov}\n Covariance Matrix of twist is here {twist_cov}")

        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov

        self.odom_pub.publish(msg)

    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    ekf_odom = EkfOdom()
    ekf_odom.spin()
    ekf_odom.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
