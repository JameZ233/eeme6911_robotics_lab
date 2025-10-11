import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


heartbeat_period = 0.1

class DoorOpener(Node):

    def __init__(self):
        super().__init__('door_opener')
        self.log = self.get_logger()
        #self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.door_pub = self.create_publisher(Float64, '/hinged_glass_door/torque', 5)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.declare_parameter ('forward_speed', 0.7)
        self.speed = self.get_parameter('forward_speed').get_parameter_value().double_value

        #State machine
        self.state = "Open_door"
        self.start_time = self.get_clock().now()

        #Subscription
        self.feature = self.create_subscription(Float64, '/feature_mean', self.heartbeat, 10)
        self.feature


    def heartbeat(self, msg):
        self.log.info('heartbeat')
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        
        if self.state == "Open_door":
            self.door_pub.publish(Float64(data=5.0))
            if msg.data <287.5:
                self._next("Move_Forward")

        elif self.state == "Move_Forward":
            twist = Twist(); twist.linear.x = self.speed
            self.cmd_pub.publish(twist)
            if elapsed >7.0:
                self._next("STOP")

        elif self.state == "STOP":
            self.cmd_pub.publish(Twist())
            self._next("Closed_door")

        elif self.state == "Closed_door":
            self.door_pub.publish(Float64(data=-5.0))
            if elapsed > 3.0:
                self.get_logger().info("Mission Done")
                #self.timer.cancel()

    def _next(self, new_state):
        self.state = new_state
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Switch to {new_state}")


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    door_opener = DoorOpener()
    door_opener.spin()
    door_opener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
