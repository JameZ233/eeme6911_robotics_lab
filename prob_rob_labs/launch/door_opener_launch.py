import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('forward', default_value = '0.70', description='Forward speed of robot'),
        Node(
            package='prob_rob_labs',
            executable='door_opener',
            name='door_opener',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'forward_speed': LaunchConfiguration('forward_speed')}]
        )
        ])
