#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for the robot teleoperation node.
    
    Returns:
        LaunchDescription: Launch configuration for the robot node
    """
    return LaunchDescription([
        Node(
            package='teleop_robot',
            executable='robot_node',
            name='robot_teleop_node',
            output='screen',
            parameters=[],
            remappings=[
                # Remap topics if needed
                # ('/cmd_vel', '/robot/cmd_vel'),
            ]
        )
    ])