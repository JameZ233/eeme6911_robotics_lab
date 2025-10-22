import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from sensor_msgs.msg import Imu
import numpy as np
import math
from message_filters import Subscriber, ApproximateTimeSynchronizer
from nav_msgs.msg import Odometry

heartbeat_period = 0.1

class EkfOdom(Node):

    def __init__(self):
        super().__init__('ekf_odom')
        self.log = self.get_logger().info("Initialized EKF Odom")
        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, '/cmd_vel',self.controlCallback, 10)
        self.twist_sub = Subscriber(self, TwistStamped, "/tb3/ground_truth/twist")
        self.pose_sub = Subscriber(self, PoseStamped, "/tb3/ground_truth/pose")
        self.imu_sub = Subscriber(self, Imu, '/imu')
        

        # Define Sync Timer
        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.pose_sub,self.twist_sub,  self.imu_sub],
                                                     queue_size=queue_size, slop=max_delay)
        self.time_sync.registerCallback(self.SyncCallback)

        # Define constants
        self.r_w = 0.033
        self.R = 0.1435/2
        self.tau_v = 1.772
        self.tau_w = 0.080
        # This is the Gain we chose
        self.Gv = 2.0
        self.Gw = 0.5

        # Define initial state
        self.x = np.zeros(5)
        self.P = np.eye(5)
        self.Q = np.diag([1e-4, 1e-4, 1e-4, 1e-3, 1e-3])
        self.R_mea = np.diag([0.05**2, 0.05**2, 0.02**2, 0.02**2, 0.02**2])

        # Define initial value from /cmd_vel
        self.u_v = 0
        self.u_w = 0

        # Publish message
        self.odom_pub=self.create_publisher(Odometry, '/ekf_odom', 10)

        # Time
        self.last_time = 0
        self.slop = self.declare_parameter('slop', 0.2).get_parameter_value().double_value

        # Define C matrix
        self.C = np.array([
            [0,0,0,1/self.r_w, self.R/self.r_w],
            [0,0,0,1/self.r_w, -self.R/self.r_w],
            [0,0,0,0,1]])


    def SyncCallback(self, pos_msg: PoseStamped, twist_msg: TwistStamped, imu_msg: Imu):
        # Read Pose Info
        x_m = pos_msg.pose.position.x
        y_m = pos_msg.pose.position.y
        pose_sec = pos_msg.header.stamp.sec
        pose_nanosec = pos_msg.header.stamp.nanosec*1e-9

        # Read imu Info
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w
        imu_sec = imu_msg.header.stamp.sec
        imu_nanosec = imu_msg.header.stamp.nanosec*1e-9
        theta_m = math.atan2(2 * qz* qw, 1-2 * qz * qz)

        # Read twist Info
        v_m = twist_msg.twist.linear.x
        w_m = twist_msg.twist.angular.z
        twist_sec = twist_msg.header.stamp.sec
        twist_nanosec = twist_msg.header.stamp.nanosec*1e-9
        self.get_logger().info(f'Sync callback with {pose_sec} and {twist_sec} as times')
        t_now = min(pose_sec+pose_nanosec, twist_sec+twist_nanosec, imu_sec+imu_nanosec)
        z=np.array([x_m, y_m, theta_m, v_m, w_m])

        # Calculate Time
        if self.last_time is None:
            self.last_time = t_now
            self.x = z.copy()
            return
        
        dt = max(1e-4, t_now-self.last_time)
        self.last_time = t_now
        # Run EKF

        self.predict(dt)
        self.update(z)

        self.publish_odom(t_now)

    def controlCallback(self, msg: Twist):
        self.u_v = msg.linear.x
        self.u_w = msg.angular.z

    # Define B matrix
    def B_matrix(self, av, aw):
        return np.array([
            [0.0,0.0],
            [0.0,0.0],
            [0.0,0.0],
            [(1.0-av)*self.Gv, 0.0],
            [0.0, (1.0-aw) * self.Gw]])

    def predict(self, dt):
        X,Y,theta,v,w = self.x

        av = 10**(-(dt / self.tau_v))
        aw = 10**(-(dt / self.tau_w))
        
        # Processing dynamics
        X1 = X + dt * v * math.cos(theta)
        Y1 = Y + dt * v * math.sin(theta)
        theta1 = theta + dt * w
        v1 = av * v + self.Gv * (1-av) * self.u_v
        w1 = aw * w + self.Gw * (1-aw) * self.u_w
        self.x = np.array([X1, Y1, theta1, v1, w1])

        F = np.array([
            [1,0,-dt * v * np.sin(theta), dt * np.cos(theta), 0],
            [0, 1, -dt * v * np.cos(theta), dt * np.sin(theta), 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, av, 0],
            [0, 0, 0, 0, aw]
            ])
        B = self.B_matrix(av, aw)

        # Noise
        Q = self.Q * dt
        self.P = F @ self.P @ F.T + Q
        self.get_logger().info(f"B=\n{B}\nC=\n{self.C}")
        

    def update(self, z):
        # Processing Measurement model
        H=np.eye(5)
        y=z-self.x
        S=self.P+self.R_mea
        K=self.P @ np.linalg.inv(S)
        self.x = self.x + K@y
        self.P = (np.eye(5) - K) @ self.P


    def publish_odom(self, t):
        X, Y, theta, v, w = self.x
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int((t-int(t))*1e-9) 

        msg.pose.pose.position.x = float(X)
        msg.pose.pose.position.y = float(Y)
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)

        # Put in covariance
        P = self.P
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36

        pose_cov[0] = float(P[0, 0])
        pose_cov[7] = float(P[1, 1])
        pose_cov[35] = float(P[2, 2])
        pose_cov[1] = float(P[0, 1])
        pose_cov[6] = float(P[1, 0])
        pose_cov[5] = float(P[0, 2]) 
        pose_cov[30] = float(P[2, 0])
        pose_cov[11] = float(P[1, 2])
        pose_cov[31] = float(P[2, 1])

        twist_cov[0] = float(P[3, 3])
        twist_cov[35] = float(P[4, 4])
        twist_cov[5] = float(P[3, 4])
        twist_cov[30] = float(P[4, 3])

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
