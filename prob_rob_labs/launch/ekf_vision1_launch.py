import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'landmark_color',
            default_value='cyan',
            description='Color of the landmark'
        ),
        DeclareLaunchArgument(
            'landmark_height',
            default_value='0.5',
            description='landmark height in meters'
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value="/home/jianzai/ros2_ws/src/eeme6911_robotics_lab/prob_rob_labs/src/ekf_filter/landmarks.yaml",
            description="Path to landmark map file"
        ),

        Node(
            package='prob_rob_labs',        
            executable='ekf_vision1',       
            name='ekf_vision1',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'landmark_color': LaunchConfiguration('landmark_color'),
                'landmark_height': LaunchConfiguration('landmark_height'),
                "map_file": LaunchConfiguration("map_file"),
            }]
        )
    ])