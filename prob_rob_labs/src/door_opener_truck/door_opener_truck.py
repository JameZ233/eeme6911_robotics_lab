import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
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
        self.declare_parameter('mode','bayes')
        self.mode = self.get_parameter('mode').value

        self.declare_parameter('previous_open', 0.5)
        self.p_open = self.get_parameter('previous_open').get_parameter_value().double_value
        self.declare_parameter('openopen', 0.9966777)
        self.oo = self.get_parameter('openopen').get_parameter_value().double_value
        self.declare_parameter('closeopen', 0.0033223)
        self.co = self.get_parameter('closeopen').get_parameter_value().double_value   
        self.declare_parameter('gothreshold', 0.99999)
        self.gothreshold = self.get_parameter('gothreshold').get_parameter_value().double_value     
        # assert self.truth in ('open', 'closed')

        #Subscription
        self.feature = self.create_subscription(Float64, '/feature_mean', self.collect_data, 10)
        self.belief = self.create_publisher(Float64, '/door_belief', 10)
        self.decision = self.create_publisher(Bool, '/door_decision', 10)
        self.feature

        self.start = time.time()
        self.sum_up = 0
        self.sum_below = 0
        self.decided = False

        self.log.info(f"Mode={self.mode}")
        if self.mode =='bayes':
            self.log.info(f"prior_open={self.p_open:.4f} openopen={self.oo:.6f} closeopen={self.co:.6f}"
                          f"decision_threshold={self.gothreshold}")
    def collect_data(self, msg):
        val = msg.data
        if self.mode == 'estimate':
            self._estimation(val)
        else:
            self._bayes_filter()
    
    def _estimation(self, val):

        if val >= self.threshold:
            self.sum_up += 1
        else:
            self.sum_below += 1


        if time.time() - self.start > self.timeframe:
            total_samples = self.sum_up + self.sum_below
            print(f"up given {self.truth} is {self.sum_up / total_samples}")
            print(f"below given {self.truth} is {self.sum_below / total_samples}")

            rclpy.shutdown()

    def _bayes_filter(self):
        z_open = True
        p = self.p_open
        if z_open:
            num = self.oo *p
            den = self.oo*p + self.co * (1-p)
        else:
            num = (1-self.oo) * p
            den = (1-self.oo) * p + (1-self.co) * (1-p)

        self.p_open = num / den if den>0 else p
        self.belief.publish(Float64(data=self.p_open))
        decision = (self.p_open >= self.gothreshold)
        self.decision.publish(Bool(data=decision))
        self.get_logger().info(f"Z = open | belief = {self.p_open:.8f} | decision to go={decision}")

        if decision:
            self.decide=True



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
