#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotTeleopNode(Node):
    """
    ROS2 node that subscribes to /cmd_vel topic and displays velocity commands.
    
    This node receives Twist messages from the web teleoperation system
    and logs the velocity values in the required format.
    """

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
        """
        Callback function for /cmd_vel topic messages.
        
        Args:
            msg (Twist): ROS2 Twist message containing velocity commands
        """
        # Extract velocity values from the message
        velocity_x = msg.linear.x
        velocity_y = msg.linear.y
        
        # Log velocity values in the required format
        self.get_logger().info(f'velocity_x={velocity_x}, velocity_y={velocity_y}')
        
        # Optional: Log additional debug information
        if msg.linear.z != 0.0 or msg.angular.x != 0.0 or msg.angular.y != 0.0 or msg.angular.z != 0.0:
            self.get_logger().debug(
                f'Additional twist components - linear.z={msg.linear.z}, '
                f'angular.x={msg.angular.x}, angular.y={msg.angular.y}, angular.z={msg.angular.z}'
            )


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create and run the robot teleop node
        robot_node = RobotTeleopNode()
        
        # Spin the node to keep it running and processing callbacks
        rclpy.spin(robot_node)
        
    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        pass
    except Exception as e:
        # Log any unexpected errors
        if 'robot_node' in locals():
            robot_node.get_logger().error(f'Unexpected error: {e}')
        else:
            print(f'Error during node initialization: {e}')
    finally:
        # Clean shutdown
        if 'robot_node' in locals():
            robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()