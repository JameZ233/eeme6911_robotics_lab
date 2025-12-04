import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
import numpy as np
import yaml

from prob_rob_msgs.msg import Point2DArrayStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped

def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    a = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return a


def color_topic(color: str) -> str:
    # This function convert to topic given a color
    return f"/vision_{color.strip().lower()}/corners"

class EkfFilter(Node):

    def __init__(self):
        super().__init__('ekf_filter')
        # Set all parameters
        self.declare_parameter('map_file', '')
        self.declare_parameter('omega_epsilon', 1e-3)   
        self.declare_parameter('process_noise_v', 0.1)  
        self.declare_parameter('process_noise_o', 0.1)
        self.declare_parameter('meas_noise_r', 0.1)     
        self.declare_parameter('meas_noise_phi', 0.05)  

        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.omega_epsilon = self.get_parameter('omega_epsilon').get_parameter_value().double_value
        q_v = self.get_parameter('process_noise_v').get_parameter_value().double_value
        q_w = self.get_parameter('process_noise_o').get_parameter_value().double_value
        r_r = self.get_parameter('meas_noise_r').get_parameter_value().double_value
        r_p = self.get_parameter('meas_noise_phi').get_parameter_value().double_value

        # Load landmarks from map file
        self.landmarks = self._load_map(map_file)
        self.colors = list(self.landmarks.keys())
        self.get_logger().info(f'Loaded {len(self.landmarks)} landmarks from {map_file}')


        # EKF state setup
        self.x = np.zeros((3, 1), dtype=float)
        self.P = np.eye(3, dtype=float) * 1e-3

        # Noise matrices
        self.q_v = q_v
        self.q_w = q_w
        self.R = np.array([[r_r**2, 0.0],
                           [0.0,    r_p**2]], dtype=float)
        
        # Last velocity setup
        self.last_v = 0.0
        self.last_omega = 0.0

        # Time setup
        self.state_time: Time | None = None

        # Subscriptions
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        # Subscribe to each landmark topic
        self.subs = []
        for color in self.colors:
            topic = f"/landmark/{color}/position"   
            sub = self.create_subscription(
                Pose2D,
                topic,
                self._measure_cb(color),
                10
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to measurements '{topic}' for color '{color}'")

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/ekf_pose', 10)

    def _load_map(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

            if "landmarks" not in data:
                raise RuntimeError("Map format is not correct")
            
            landmarks_raw = data["landmarks"]
            landmarks = {} 

        for color_name, info in landmarks_raw.items():
            x = float(info["x"])
            y = float(info["y"])

            landmarks[color_name.strip().lower()] = {"x": x, "y": y}

        return landmarks
    
    def _odom_cb(self, msg: Odometry):
        # Get velocities
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z

        t = Time.from_msg(msg.header.stamp)
        if self.state_time is None:
            self.state_time = t
            self.last_v = v
            self.last_omega = omega
            self.get_logger().info("EKF time initialized.")
            return

        dt = (t - self.state_time).nanoseconds * 1e-9
        if dt < 0.0:
            # Discard case
            self.get_logger().warn(f"Sampling time (dt={dt:.3f} < 0), discarded.")
            return

        if dt > 0.0:
            # Predict case
            self._predict(v, omega, dt)
            self.state_time = t

        self.last_v = v
        self.last_omega = omega

        # Publish pose
        # self._publish_pose(msg.header.stamp)
    
    def _measure_cb(self, color):
        def cb(msg: Pose2D):
            r_meas = msg.x
            phi_meas = msg.theta

            t = Time.from_msg(self.get_clock().now().to_msg())

            if self.state_time is None:
                # Set time for first measurement
                self.state_time = t
                self.get_logger().info("EKF time initialized.")
                return

            dt = (t - self.state_time).nanoseconds * 1e-9
            if dt < 0.0:
                self.get_logger().warn(
                    f"Measurement for {color}: (dt={dt:.3f} < 0), discarded."
                )
                return

            if dt > 0.0:
                self._predict(self.last_v, self.last_omega, dt)

            landmark = self.landmarks[color]
            lx = landmark["x"]
            ly = landmark["y"]

            self._update(r_meas, phi_meas, lx, ly)

            self.state_time = t

            # Publish
            self._publish_pose(self.get_clock().now().to_msg())
        return cb
    

    def _predict(self, v: float, omega: float, dt: float):
        x, y, theta = self.x.flatten()
        w = omega

        if abs(w) < self.omega_epsilon:
            dx = v * dt * math.cos(theta)
            dy = v * dt * math.sin(theta)
            dtheta = 0.0
        else:
            dx = (-v / w) * math.sin(theta) + (v / w) * math.sin(theta + w * dt)
            dy = ( v / w) * math.cos(theta) - (v / w) * math.cos(theta + w * dt)
            dtheta = w * dt

        x_new = x + dx
        y_new = y + dy
        theta_new = normalize_angle(theta + dtheta)
        self.x = np.array([[x_new], [y_new], [theta_new]], dtype=float)

        # Jacobian F
        F = np.eye(3, dtype=float)
        F[0, 2] = -v * dt * math.sin(theta) if abs(w) < self.omega_epsilon else 0.0
        F[1, 2] =  v * dt * math.cos(theta) if abs(w) < self.omega_epsilon else 0.0

        # Process noise covariance
        q_v = self.q_v
        q_w = self.q_w
        Q = np.diag([
            (q_v * abs(v) * dt)**2,
            (q_v * abs(v) * dt)**2,
            (q_w * abs(w) * dt)**2
        ])

        self.P = F @ self.P @ F.T + Q



    def _update(self, r_meas: float, phi_meas: float, lx: float, ly: float):
        x, y, theta = self.x.flatten()

        dx = lx - x
        dy = ly - y

        # Expected from current state
        r_hat = math.sqrt(dx**2 + dy**2)
        phi_hat = normalize_angle(math.atan2(dy, dx) - theta)

        # Measurement residual
        z = np.array([[r_meas],
                      [phi_meas]], dtype=float)
        h = np.array([[r_hat],
                      [phi_hat]], dtype=float)
        y_res = z - h
        y_res[1, 0] = normalize_angle(y_res[1, 0])

        if r_hat < 1e-6:
            self.get_logger().warn("r_hat too small during update, skipping.")
            return

        H = np.zeros((2, 3), dtype=float)
        H[0, 0] = -dx / r_hat
        H[0, 1] = -dy / r_hat
        H[0, 2] = 0.0

        H[1, 0] =  dy / (r_hat**2)
        H[1, 1] = -dx / (r_hat**2)
        H[1, 2] = -1.0

        # Kalman gain
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # State and covariance update
        self.x = self.x + K @ y_res
        self.x[2, 0] = normalize_angle(self.x[2, 0])

        I = np.eye(3, dtype=float)
        self.P = (I - K @ H) @ self.P


    def _publish_pose(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "world" 

        x, y, theta = self.x.flatten()

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Covariance Matrix
        cov = np.zeros((6, 6), dtype=float)
        cov[0, 0] = self.P[0, 0]
        cov[0, 1] = self.P[0, 1]
        cov[1, 0] = self.P[1, 0]
        cov[1, 1] = self.P[1, 1]
        cov[5, 5] = self.P[2, 2] 

        msg.pose.covariance = cov.flatten().tolist()

        self.pose_pub.publish(msg)
    
    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_filter = EkfFilter()
    ekf_filter.spin()
    ekf_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
