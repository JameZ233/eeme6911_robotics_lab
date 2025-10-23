import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
import math


heartbeat_period = 0.1

class OdomEval(Node):

    def __init__(self):
        super().__init__('odom_eval')
        self.log = self.get_logger().info('Initialzing odom evaluation')
        self.pose_sub = Subscriber(self, PoseStamped, '/tb3/ground_truth/pose')
        self.ekf_sub = Subscriber(self, Odometry, '/ekf_odom')

        # Define Sync Timer
        queue_size = 10
        max_delay = 0.02
        self.time_sync = ApproximateTimeSynchronizer([self.pose_sub,self.ekf_sub],
                                                     queue_size=queue_size, slop=max_delay)
        self.time_sync.registerCallback(self.SyncCallback)

        # Error Publish
        self.error_pub = self.create_publisher(Float64MultiArray, "/ekf_error", 10)

    
    def SyncCallback(self, pose_msg: PoseStamped, ekf_msg: Odometry):
        # Ground Truth 
        x_gt = pose_msg.pose.position.x
        y_gt = pose_msg.pose.position.y
        theta_gt = math.atan2(pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)

        # EKF odom
        x_ekf = ekf_msg.pose.pose.position.x
        y_ekf = ekf_msg.pose.pose.position.y
        theta_ekf = math.atan2(ekf_msg.pose.pose.orientation.z, ekf_msg.pose.pose.orientation.w)

        # Calculating position Error
        error_pos = math.sqrt((x_gt - x_ekf)**2 + (y_gt - y_ekf)**2)

        # Calculating angular Error
        dtheta = theta_gt - theta_ekf
        error_theta = abs(math.atan2(math.sin(dtheta), math.cos(dtheta)))

        # Publish msg
        msg = Float64MultiArray()
        msg.data = [error_pos, error_theta]
        self.error_pub.publish(msg)

        


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odom_eval = OdomEval()
    odom_eval.spin()
    odom_eval.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
