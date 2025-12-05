import rclpy
from rclpy.node import Node
from rclpy.time import Time

import math
import numpy as np
import yaml
from functools import partial

from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose2D
from prob_rob_msgs.msg import Point2DArrayStamped

# Helper functions remain unchanged
def estimate(points):
    # Parse corner points from input dictionary
    pts = [(float(p.x), float(p.y)) for p in points] if points else []
    if len(pts) < 4:
        return None, None
    
    xs = np.array([pt[0] for pt in pts], dtype=float)
    ys = np.array([pt[1] for pt in pts], dtype=float)

    # Compute Height
    y_top, y_bottom = np.max(ys), np.min(ys)
    height = y_top - y_bottom

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

def color_topic(color) -> str:
    return f"/vision_{color.strip().lower()}/corners"

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
        self.declare_parameter("landmark_height", 0.5)
        self.declare_parameter("map_file", "")

        self.landmark_height = self.get_parameter("landmark_height").get_parameter_value().double_value
        map_file = self.get_parameter("map_file").get_parameter_value().string_value
        self.landmarks = self._load_map(map_file)
        self.colors = list(self.landmarks.keys())
        self.get_logger().info(f"Loaded {len(self.colors)} landmarks from map: {self.colors}")

        # Minimum parameters for stop detection
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
        self.last_seen = {c: None for c in self.colors} 
        self.was_visible = {c: False for c in self.colors}    

        # Subscriptions for camera and landmarks
        self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info, 10)
        self._subs = []
        self.pub_pose = {}
        
        for color in self.colors:
            topic = color_topic(color)
            cb = partial(self._handle_corners, color=color)
            sub = self.create_subscription(
                Point2DArrayStamped,
                topic,
                cb,
                10
            )
            self._subs.append(sub)

            # Publisher
            pose_topic = f"/landmark/{color}/position"
            self.pub_pose[color] = self.create_publisher(Pose2D, pose_topic, 10)

            self.get_logger().info(f"Color '{color}': subscribe {topic}, publish {pose_topic}")

        # Timer to check timeout
        self.create_timer(0.1, self._visibility_check)
    
    def _load_map(self, path: str):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        landmarks_raw = data["landmarks"]
        landmarks = {}
        for color, info in landmarks_raw.items():
            x = float(info["x"])
            y = float(info["y"])
            landmarks[color.strip().lower()] = {"x": x, "y": y}
        return landmarks
    
    def camera_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        try:
            if msg.width > 0:
                self.img_width = int(msg.width)
        except:
            pass

    def _handle_corners(self, msg: Point2DArrayStamped, color: str):
        if self.fx is None or self.fy is None:
            # Wait until we have camera intrinsics
            self.get_logger().warn_once("CameraInfo not received yet.")
            return
        
        x_axis, height = estimate(msg.points)
        
        state, info = is_visible(
            points=msg.points,
            h_px=height,
            x_axis_px=x_axis,
            img_width=self.img_width,
            min_points=self.min_points,
            min_height_px=self.min_height_px
        )

        if not state:
            if self.was_visible[color]:
                self.get_logger().warn(f"{color} Landmark not visible {info}. Stopping publish.")
            self.was_visible[color] = False
            self.last_seen[color] = None
        else:
            theta = math.atan((self.cx - x_axis) / self.fx)
            d = (self.landmark_height * self.fy) / (height * math.cos(theta)) 

            # Publish
            out = Pose2D()
            out.x = float(d)   
            out.y = 0.0            
            out.theta = float(theta)
            self.pub_pose[color].publish(out)

            self.last_seen[color] = self.get_clock().now()
            
            if not self.was_visible[color]:
                self.get_logger().info(f"{color} Landmark visible again.")
            self.was_visible[color] = True

            self.get_logger().info(
                f"{color} distance={d:.2f} m | bearing={math.degrees(theta):.2f} deg | "
                f"height={height:.1f}, x_axis_pos={x_axis:.1f}"
            )

    def _visibility_check(self):
        now = self.get_clock().now()
        for color in self.colors:
            last = self.last_seen[color]
            if last is None:
                continue
            age = (now - last).nanoseconds * 1e-9
            if age > self.timeout_sec and self.was_visible[color]:
                self.was_visible[color] = False
                self.last_seen[color] = None
                self.get_logger().warn(
                    f"{color} No corners for {age:.2f}s (> {self.timeout_sec}s). Stopping publish."
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