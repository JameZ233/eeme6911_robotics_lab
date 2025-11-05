import rclpy
from rclpy.node import Node
from rclpy.time import Time

import math
import numpy as np

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose2D
from prob_rob_msgs.msg import Point2DArrayStamped


# Height and axis estimation function
def estimate(points):
    # Parse corner points from input dictionary
    xs = np.array([float(p.x) for p in points], dtype=float)
    ys = np.array([float(p.y) for p in points], dtype=float)

    # Compute Height
    y_top, y_bottom = np.min(ys), np.max(ys)
    height = y_bottom - y_top

    # Split left and right points using median x
    x_med = np.median(xs)
    left = np.stack([xs[xs <= x_med], ys[xs <= x_med]], axis=1)
    right = np.stack([xs[xs >  x_med], ys[xs >  x_med]], axis=1)
    if left.size == 0 or right.size == 0 or height <= 0.0:
        return float("nan"), 0.0

    # Sort by y and pair points by rank
    L = left[left[:,1].argsort()]
    R = right[right[:,1].argsort()]
    k = min(len(L), len(R))
    if k == 0:
        return float("nan"), 0.0
    L, R = L[:k], R[:k]

    # Compute midpoints and take median for axis
    mid_x = 0.5 * (L[:,0] + R[:,0])
    x_axis = np.median(mid_x)

    return x_axis, height

def color(color) -> str:
    return f"/vision_{color.strip().lower()}/corners"


# Function to check visibility criteria
def is_visible(points, h_px, x_axis_px, img_width, min_points, min_height_px):
    
    # Check number of points
    if points is None or len(points) < min_points:
        return False, "too few corner points"

    # Check height
    if not np.isfinite(h_px) or h_px < min_height_px:
        return False, f"pixel height too small ({h_px:.1f})"

    # Check x_axis within image bounds
    if img_width is not None:
        if not np.isfinite(x_axis_px) or x_axis_px < -5 or x_axis_px > (img_width + 5):
            return False, f"x_axis outside image ({x_axis_px:.1f})"

    return True, ""

class EkfVision1(Node):

    def __init__(self):
        super().__init__('ekf_vision1')
        # Parameters
        self.declare_parameter("landmark_color", "cyan")  
        self.declare_parameter("landmark_height", 0.5)

        self.landmark_color = self.get_parameter("landmark_color").get_parameter_value().string_value
        self.landmark_height = self.get_parameter("landmark_height").get_parameter_value().double_value

        # Minimum parameters for calculation
        self.declare_parameter("min_points", 6)      
        self.declare_parameter("min_height_px", 8.0) 
        self.declare_parameter("timeout_sec", 0.75)  

        self.min_points = self.get_parameter("min_points").get_parameter_value().integer_value
        self.min_height_px = self.get_parameter("min_height_px").get_parameter_value().double_value
        self.timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value

        # Camera Info setup
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.img_width = None 

        # Last time corners were seen
        self.last_seen: Time | None = None
        self.was_visible = False 

        # Subscriptions
        self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info, 10)
        self.create_subscription(Point2DArrayStamped, color(self.landmark_color), self.corners_cb, 10)

        # Publisher
        self.pub_pose = self.create_publisher(Pose2D, "/landmark/relative_pose", 10)

        # Timer to check timeout
        self.create_timer(0.1, self._visibility_check)

        self.get_logger().info(
            f"Listening to: {'/camera/camera_info'} and {color(self.landmark_color)} | "
            f"height={self.landmark_height:.3f} m, color='{self.landmark_color}'"
        )
    
    
    def camera_info(self, msg: CameraInfo):
        # Read from camera info message
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        try:
            if msg.width > 0:
                self.img_width = int(msg.width)
        except:
            pass

        

    def corners_cb(self, msg: Point2DArrayStamped):
        if self.fx is None or self.fy is None:
            # Wait until we have camera intrinsics
            self.get_logger().warn("CameraInfo not received yet.")
            return

        
        height, x_axis = estimate(msg.points)
        state, info = is_visible(
            points=msg.points,
            h_px=height,
            x_axis_px=x_axis,
            img_width=self.img_width,
            min_points=self.min_points,
            min_height_px=self.min_height_px
        )

        if not state:
            if self.was_visible:  # only log once when transitioning to not-visible
                self.get_logger().warn(f"Landmark not visible {info}. Stopping publish.")
            self.was_visible = False
            self.last_seen = None
            return
        
        # Bearing angle
        theta = math.atan((self.cx - x_axis) / self.fx)
        # Distance
        d = (self.landmark_height * self.fy) / (height * math.cos(theta)) 


        # Publish
        out = Pose2D()
        out.x = float(d)   
        out.y = 0.0            
        out.theta = float(theta)
        self.pub_pose.publish(out)

        self.last_seen = self.get_clock().now()
        if not self.was_visible:
            self.get_logger().info("Landmark visible again.")
        self.was_visible = True

        self.get_logger().info(
            f"distance={d:.2f} m | bearing={math.degrees(theta):.2f} deg | "
            f"height={height:.1f}, x_axis_pos={x_axis:.1f}"
        )


    def _visibility_check(self):

        if self.last_seen is None:
            return
        age = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
        if age > self.timeout_sec and self.was_visible:
            self.was_visible = False
            self.last_seen = None
            self.get_logger().warn(
                f"No corners for {age:.2f}s (> {self.timeout_sec}s). Stopping publish."
            )


    def spin(self):
        rclpy.spin(self)
    


def main():
    rclpy.init()
    ekf_vision1 = EkfVision1()
    ekf_vision1.spin()
    ekf_vision1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
