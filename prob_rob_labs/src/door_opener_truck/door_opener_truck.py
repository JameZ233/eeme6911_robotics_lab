import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time


heartbeat_period = 0.1

class DoorThresholdStatistics(Node):

    def __init__(self):
        super().__init__('door_threshold_stats')
        self.log = self.get_logger()

        self.declare_parameter ('threshold', 240.0)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.declare_parameter ('window_time', 0.1)
        self.windowtime = self.get_parameter('window_time').get_parameter_value().double_value
        self.declare_parameter ('time_frame', 10.0)
        self.timeframe = self.get_parameter('time_frame').get_parameter_value().double_value
        self.declare_parameter ('truth', 'closed')
        self.truth = self.get_parameter('truth').value
        # assert self.truth in ('open', 'closed')

        #Subscription
        self.door_pub = self.create_publisher(Float64, '/hinged_glass_door/torque', 5)
        self.feature = self.create_subscription(Float64, '/feature_mean', self.collect_data, 10)
        self.feature

        self.start = time.time()
        self.sum_up = 0
        self.sum_below = 0


    def collect_data(self, msg):
        val = msg.data
        if val >= self.threshold:
            self.sum_up += 1
        else:
            self.sum_below += 1


        if time.time() - self.start > self.timeframe:
            total_samples = self.sum_up + self.sum_below
            print(f"up given {self.truth} is {self.sum_up / total_samples}")
            print(f"below given {self.truth} is {self.sum_below / total_samples}")

            rclpy.shutdown()


    # def heartbeat(self, msg):
    #     self.log.info('heartbeat')
    #     elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        
    #     if self.state == "Open_door":
    #         self.door_pub.publish(Float64(data=5.0))
    #         if msg.data <265.0:
    #             self._next("Move_Forward")

    #     elif self.state == "Move_Forward":
    #         twist = Twist(); twist.linear.x = self.speed
    #         self.cmd_pub.publish(twist)
    #         if elapsed >7.0:
    #             self._next("STOP")

    #     elif self.state == "STOP":
    #         self.cmd_pub.publish(Twist())
    #         self._next("Closed_door")

    #     elif self.state == "Closed_door":
    #         self.door_pub.publish(Float64(data=-5.0))
    #         if elapsed > 3.0:
    #             self.get_logger().info("Mission Done")
    #             #self.timer.cancel()

    def _next(self, new_state):
        self.state = new_state
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Switch to {new_state}")


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    door_opener = DoorThresholdStatistics()
    door_opener.spin()
    door_opener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
