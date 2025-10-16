import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

heartbeat_period = 0.1

class EkfOdom(Node):

    def __init__(self):
        super().__init__('ekf_odom')
        self.log = self.get_logger()
        qos = QoSProfile(depth=10)
        self.twist_sub = Subscriber(self, TwistStamped, "/tb3/ground_truth/twist")
        self.pose_sub = Subscriber(self, PoseStamped, "/tb3/ground_truth/pose")

        self.timer = self.create_timer(1, self.TimerCallback)
        self.second_timer = self.create_timer(1.05, self.SecondTimerCallback)

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.twist_sub, self.pose_sub],
                                                     queue_size, max_delay)
        self.time_sync.registerCallback(self.SyncCallback)
        r_w = 0.033
        R = 0.1435/2
        a_v = 0.997404
        a_w = 0.943942
        self.C = np.array([
            [0,0,0,1/r_w, R/r_w],
            [0,0,0,1/r_w, -R/r_w],
            [0,0,0,0,1]])


    def SyncCallback(self, pose, twist):
        pose_sec = pose.header.stamp.sec
        twist_sec = twist.header.stamp.sec
        self.get_logger().info(f'Sync callback with {pose_sec} and {twist_sec} as times')
        if (pose.header.stamp.sec > 2.0):
            new_fluid = FluidPressure()
            new_fluid.header.stamp = Clock().now().to_msg()
            new_fluid.header.frame_id = 'test'
            new_fluid.fluid_pressure = 2.5
            self.fluid_pub.publish(new_fluid)

    def TimerCallback(self):
        temp = Temperature()
        self.now = Clock().now().to_msg()

        temp.header.stamp = self.now
        temp.header.frame_id = 'test'
        temp.temperature = 1.0
        self.temp_pub.publish(temp)

    def SecondTimerCallback(self):
        fluid = FluidPressure()
        self.now = Clock().now().to_msg()

        fluid.header.stamp = self.now
        fluid.header.frame_id = "test"
        fluid.fluid_pressure = 2.0
        self.fluid_pub.publish(fluid)   

    # def heartbeat(self):
    #     self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)
    
    def Jacobian(self, x, dt):
        X,Y,theta,v,w = x
        F = np.array([
            [1,0,-dt * v * np.sin(theta), dt * np.cos(theta), 0],
            [0, 1, -dt * v * np.cos(theta), dt * np.sin(theta), 0],
            [0, 0, 1, 0, dt],
            [0, 0, 0, self.a_v, 0],
            [0, 0, 0, 0, self.a_w]
            ])
        return F
    
    def B_matrix(self):
        B = np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [(1-self.a_v) * 2.0, 0],
            [0, (1-self.a_w) * 0.5]
            ])
        return B
    
    


def main():
    rclpy.init()
    ekf_odom = EkfOdom()
    ekf_odom.spin()
    ekf_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
