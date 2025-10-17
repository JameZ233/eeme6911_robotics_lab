import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
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
        self.create_subscription('/cmd_vel',self.controlCallback, 10)
        self.twist_sub = Subscriber(self, TwistStamped, "/tb3/ground_truth/twist")
        self.pose_sub = Subscriber(self, PoseStamped, "/tb3/ground_truth/pose")
        

        # Define Sync Timer
        # self.timer = self.create_timer(1, self.TimerCallback)
        # self.second_timer = self.create_timer(1.05, self.SecondTimerCallback)

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.twist_sub, self.pose_sub],
                                                     queue_size, max_delay)
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

        self.C = np.array([
            [0,0,0,1/r_w, R/r_w],
            [0,0,0,1/r_w, -R/r_w],
            [0,0,0,0,1]])


    def SyncCallback(self, pos_msg: PoseStamped, twist_msg: TwistStamped):
        # Read Pose Info
        x_m = pos_msg.pose.position.x
        y_m = pos_msg.pose.position.y
        qz = pos_msg.pose.orientation.z
        qw = pos_msg.pose.orientation.w
        pose_sec = pos_msg.header.stamp.sec
        pose_nanosec = pos_msg.header.stamp.nanosec*1e-9
        theta_m = math.atan2(2 * qz* qw, 1-2 * qz * qz)

        # Read twist Info
        v_m = twist_msg.twist.linear.x
        w_m = twist_msg.twist.angular.z
        twist_sec = twist_msg.header.stamp.sec
        twist_nanosec = twist_msg.header.stamp.nanosec*1e-9
        self.get_logger().info(f'Sync callback with {pose_sec} and {twist_sec} as times')
        t_now = min(pose_sec+pose_nanosec, twist_sec+twist_nanosec)
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

    # def heartbeat(self):
    #     self.log.info('heartbeat')

    def predict(self, dt):
        X,Y,theta,v,w = self.x

        av = 10**(-(dt / self.tau_v))
        aw = 10**(-(dt / self.tau_w))

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
        Q = self.Q * dt
        self.P = F @ self.P @ F.T + Q

        # B = np.array([
        #     [0, 0],
        #     [0, 0],
        #     [0, 0],
        #     [(1-av) * 2.0, 0],
        #     [0, (1-aw) * 0.5]
        #     ])
        

    def update(self, z):
        pass


    def publish_odom(self, t):
        pass

    def spin(self):
        rclpy.spin(self)
    
    
    def B_matrix(self):
        
        return B
    
    


def main():
    rclpy.init()
    ekf_odom = EkfOdom()
    ekf_odom.spin()
    ekf_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
