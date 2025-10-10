import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('frame_id', default_value='odom',
                              description='frame id for published ground truth'),
        DeclareLaunchArgument('link_name', default_value='waffle_pi::based_footpint',
                              description='Link Name to extract odometry'),
        Node(
            package='prob_rob_labs',
            executable='odom',
            name='odom',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'frame_id': LaunchConfiguration('frame_id')},
                        {'link_name': LaunchConfiguration('link_name')}]
        )
    ])
