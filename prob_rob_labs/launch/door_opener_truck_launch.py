import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('threshold', default_value = '240.0', description='Threshold of the door state'),
        DeclareLaunchArgument('timeframe', default_value = '10.0', description='Length of time'),
        DeclareLaunchArgument('truth', default_value = 'close', description='State of the door'),
        Node(
            package='prob_rob_labs',
            executable='door_opener_truck',
            name='door_opener_truck',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'threshold': LaunchConfiguration('threshold')},
                        {'timeframe': LaunchConfiguration('timeframe')},
                        {'truth': LaunchConfiguration('truth')}]
        )
    ])
