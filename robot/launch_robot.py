#!/usr/bin/env python3
"""
Launch script for robot node using ROS2 launch system
"""
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotTeleopNode(Node):
    def __init__(self):
        super().__init__('robot_teleop_node')
        
        # Create subscription to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10  # QoS history depth
        )
        
        self.get_logger().info('Robot teleoperation node started')
        self.get_logger().info('Subscribing to /cmd_vel topic')

    def cmd_vel_callback(self, msg):
        # Extract velocity values from the message
        velocity_x = msg.linear.x
        velocity_y = msg.linear.y
        
        # Log velocity values in the required format
        self.get_logger().info(f'velocity_x={velocity_x}, velocity_y={velocity_y}')

def main():
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run the robot teleop node
        robot_node = RobotTeleopNode()
        
        # Spin the node to keep it running and processing callbacks
        rclpy.spin(robot_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Clean shutdown
        try:
            if 'robot_node' in locals():
                robot_node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()