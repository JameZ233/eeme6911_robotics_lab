import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseStamped, TwistStamped



heartbeat_period = 0.1

class Odom(Node):

    def __init__(self):
        super().__init__('odom')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.odom = self.create_subscription(LinkStates, '/gazebo/link_states', self.listener_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/tb3/ground_truth/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/tb3/ground_truth/twist', 10)


    def listener_callback(self, msg):
        idx = msg.name.index('waffle_pi::base_footprint')

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose = pose

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'odom'
        twist_msg.twist = twist


        self.pose_pub.publish(pose_msg)
        self.twist_pub.publish(twist_msg)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    odom = Odom()
    odom.spin()
    odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
