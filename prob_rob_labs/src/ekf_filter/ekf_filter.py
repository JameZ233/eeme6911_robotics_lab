import rclpy
from rclpy.node import Node
import os
import yaml

from prob_rob_msgs.msg import Point2DArrayStamped


def color_topic(color: str) -> str:
    # This function convert to topic given a color
    return f"/vision_{color.strip().lower()}/corners"

class EkfFilter(Node):

    def __init__(self):
        super().__init__('ekf_filter')
        # Set map file
        self.declare_parameter('map_file', '')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        # Load landmarks from map file
        self.landmarks = self._load_map(map_file)
        self.get_logger().info(f'Loaded {len(self.landmarks)} landmarks from {map_file}')
        # Subscribe to each landmark topic
        self.subs = []
        for color_name in self.landmarks.keys():
            topic = color_topic(color_name)
            sub = self.create_subscription(
                Point2DArrayStamped,
                topic,
                self._make_corner_callback(color_name),
                10
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed to topic '{topic}' for color '{color_name}'")

    def _load_map(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

            if "landmarks" not in data:
                raise RuntimeError("Map file must contain a top-level 'landmarks' key")
            
            landmarks_raw = data["landmarks"]
            landmarks = {} 

        for color_name, info in landmarks_raw.items():
            x = float(info["x"])
            y = float(info["y"])

            landmarks[color_name.strip().lower()] = {"x": x, "y": y}

        return landmarks
    
    def _make_corner_callback(self, color_name):
        def cb(msg: Point2DArrayStamped):
            landmark = self.landmarks.get(color_name)

            x = landmark["x"]
            y = landmark["y"]

            num_pts = len(msg.points)
            self.get_logger().info(
                f"[{color_name}] corners received: {num_pts} pts | "
                f"landmark map pose=({x:.2f}, {y:.2f})"
            )
        return cb
    
    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    ekf_filter = EkfFilter()
    ekf_filter.spin()
    ekf_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
