import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Float64, Bool, Empty
from geometry_msgs.msg import Twist

heartbeat_period = 0.1

class BayesEstimator(Node):

    def __init__(self):
        super().__init__('bayes_estimator')
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

        self.declare_parameter('openopen', 0.9966777)
        self.oo = self.get_parameter('openopen').get_parameter_value().double_value
        self.declare_parameter('closeopen', 0.0033223)
        self.co = self.get_parameter('closeopen').get_parameter_value().double_value   
        self.declare_parameter('gothreshold', 0.99)
        self.gothreshold = self.get_parameter('gothreshold').get_parameter_value().double_value
        self.declare_parameter('com_suc', 0.2)
        self.p_cmd = self.get_parameter('com_suc').get_parameter_value().double_value  
        self.declare_parameter('closeclose', 0.97333333)
        self.cc = self.get_parameter('closeclose').get_parameter_value().double_value
        self.declare_parameter('openclose', 0.02666667)
        self.oc = self.get_parameter('openclose').get_parameter_value().double_value     
        # assert self.truth in ('open', 'closed')

        #Subscription
        self.feature = self.create_subscription(Float64, '/feature_mean', self.cb_feature, 10)
        self.pub_belief = self.create_publisher(Float64, '/door_belief', 10)
        self.pub_decision = self.create_publisher(Bool, '/door_decision', 10)
        self.pub_open = self.create_publisher(Empty, '/door_open', 10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.bayes_filter)
        self.belief = [0.5, 0.5]
        self.last_feature_mean = None
        self.try_count = 0
        self.max_trials = 10
        self.feature
        self.close = 0

        self.start = time.time()
        self.sum_up = 0
        self.sum_below = 0
        self.decided = False

        # self.log.info(f"Mode={self.mode}")
        # if self.mode =='bayes':
        #     self.log.info(f"prior_open={self.p_open:.4f} openopen={self.oo:.6f} closeopen={self.co:.6f}"
        #                   f"decision_threshold={self.gothreshold}")
            
    def cb_feature(self, msg):
        self.last_feature_mean = float(msg.data)

    def predict(self, use_cmd):
        po, pc = self.belief
        p = self.p_cmd
        e = self.close

        if use_cmd:
            po_new = 1.0 * po + p * pc
            pc_new = 0.0 * po + (1-p) * pc
        else:
            po_new = (1-e) * po
            pc_new = e*po + 1.0 * pc

        s= po_new + pc_new
        self.belief = [po_new / s, pc_new/s]


    def correct(self, z_open):
        po, pc = self.belief
        if z_open:
            like_o = self.oo
            like_c = self.co
        else:
            like_o = self.oc
            like_c = self.cc

        po_new = like_o * po
        pc_new = like_c * pc
        s= po_new + pc_new 
        self.belief = [po_new / s, pc_new/s]
        
    def decide(self):
        if self.last_feature_mean is None:
            return None
        return self.last_feature_mean <= self.threshold
    
    def move_forward(self, v=0.5):
        msg= Twist()
        msg.linear.x = v
        self.pub_vel.publish(msg)

    def stop(self):
        self.pub_vel.publish(Twist())

    def bayes_filter(self):
        p_open = self.belief[0]
        self.get_logger().info(f"Open belief {p_open}")
        if p_open >= self.gothreshold:
            self.move_forward()
            return
        if self.try_count <= self.max_trials:
            self.try_count +=1
            self.pub_open.publish(Empty())
            self.predict(use_cmd=True)
        else:
            self.predict(use_cmd=False)

        z=self.decide()
        if z is not None:
            self.correct(z_open=z)


    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    bayes_estimator = BayesEstimator()
    bayes_estimator.spin()
    bayes_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
