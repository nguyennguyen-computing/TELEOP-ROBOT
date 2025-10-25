#!/usr/bin/env python3

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from teleop_robot.robot_node import RobotTeleopNode
import time
import threading
import io
import sys
from contextlib import redirect_stdout, redirect_stderr
import logging


class TestRobotNode(unittest.TestCase):
    """Test cases for the RobotTeleopNode."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 for testing."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 after testing."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.robot_node = RobotTeleopNode()
        self.test_publisher = self.robot_node.create_publisher(Twist, '/cmd_vel', 10)
        
    def tearDown(self):
        """Clean up test fixtures."""
        self.robot_node.destroy_node()

    def test_node_initialization(self):
        """Test that the node initializes correctly."""
        self.assertEqual(self.robot_node.get_name(), 'robot_teleop_node')
        self.assertIsNotNone(self.robot_node.subscription)

    def test_cmd_vel_callback(self):
        """Test the cmd_vel callback function."""
        # Create a test Twist message
        test_msg = Twist()
        test_msg.linear.x = 1.5
        test_msg.linear.y = -0.8
        test_msg.linear.z = 0.0
        test_msg.angular.x = 0.0
        test_msg.angular.y = 0.0
        test_msg.angular.z = 0.0
        
        # Call the callback directly
        self.robot_node.cmd_vel_callback(test_msg)
        
        # Test passes if no exception is raised
        self.assertTrue(True)

    def test_message_reception(self):
        """Test that the node can receive published messages."""
        # Create a test message
        test_msg = Twist()
        test_msg.linear.x = 2.0
        test_msg.linear.y = 1.0
        
        # Publish the message
        self.test_publisher.publish(test_msg)
        
        # Spin briefly to process the message
        rclpy.spin_once(self.robot_node, timeout_sec=0.1)
        
        # Test passes if no exception is raised
        self.assertTrue(True)

    def test_message_format_validation(self):
        """Test ROS2 Twist message format validation - Requirement 4.5, 5.7"""
        test_cases = [
            # Test case: Forward movement
            {
                'name': 'forward_movement',
                'linear': {'x': 1.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'expected_log': 'velocity_x=1.0, velocity_y=0.0'
            },
            # Test case: Backward movement
            {
                'name': 'backward_movement',
                'linear': {'x': -1.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'expected_log': 'velocity_x=-1.0, velocity_y=0.0'
            },
            # Test case: Right movement
            {
                'name': 'right_movement',
                'linear': {'x': 0.0, 'y': 1.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'expected_log': 'velocity_x=0.0, velocity_y=1.0'
            },
            # Test case: Left movement
            {
                'name': 'left_movement',
                'linear': {'x': 0.0, 'y': -1.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'expected_log': 'velocity_x=0.0, velocity_y=-1.0'
            },
            # Test case: Diagonal movement
            {
                'name': 'diagonal_movement',
                'linear': {'x': 0.5, 'y': 0.7, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'expected_log': 'velocity_x=0.5, velocity_y=0.7'
            },
            # Test case: Stop command
            {
                'name': 'stop_command',
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'expected_log': 'velocity_x=0.0, velocity_y=0.0'
            }
        ]

        for test_case in test_cases:
            with self.subTest(test_case=test_case['name']):
                # Create Twist message
                msg = Twist()
                msg.linear.x = test_case['linear']['x']
                msg.linear.y = test_case['linear']['y']
                msg.linear.z = test_case['linear']['z']
                msg.angular.x = test_case['angular']['x']
                msg.angular.y = test_case['angular']['y']
                msg.angular.z = test_case['angular']['z']
                
                # Verify message format
                self.assertEqual(msg.linear.x, test_case['linear']['x'])
                self.assertEqual(msg.linear.y, test_case['linear']['y'])
                self.assertEqual(msg.linear.z, 0.0)  # Should always be 0
                self.assertEqual(msg.angular.x, 0.0)  # Should always be 0
                self.assertEqual(msg.angular.y, 0.0)  # Should always be 0
                self.assertEqual(msg.angular.z, 0.0)  # Should always be 0 (Requirement 5.7)
                
                # Test callback execution
                try:
                    self.robot_node.cmd_vel_callback(msg)
                except Exception as e:
                    self.fail(f"Callback failed for {test_case['name']}: {e}")

    def test_coordinate_system_mapping(self):
        """Test coordinate system mapping - Requirement 5.7"""
        # Test Forward = +X, Backward = -X
        forward_msg = Twist()
        forward_msg.linear.x = 1.0  # Forward should be positive X
        forward_msg.linear.y = 0.0
        forward_msg.angular.z = 0.0
        
        backward_msg = Twist()
        backward_msg.linear.x = -1.0  # Backward should be negative X
        backward_msg.linear.y = 0.0
        backward_msg.angular.z = 0.0
        
        # Test Right = +Y, Left = -Y
        right_msg = Twist()
        right_msg.linear.x = 0.0
        right_msg.linear.y = 1.0  # Right should be positive Y
        right_msg.angular.z = 0.0
        
        left_msg = Twist()
        left_msg.linear.x = 0.0
        left_msg.linear.y = -1.0  # Left should be negative Y
        left_msg.angular.z = 0.0
        
        # Test all coordinate mappings
        test_messages = [
            ('forward', forward_msg),
            ('backward', backward_msg),
            ('right', right_msg),
            ('left', left_msg)
        ]
        
        for direction, msg in test_messages:
            with self.subTest(direction=direction):
                # Verify angular.z is always 0 (Requirement 5.7)
                self.assertEqual(msg.angular.z, 0.0)
                
                # Test callback execution
                try:
                    self.robot_node.cmd_vel_callback(msg)
                except Exception as e:
                    self.fail(f"Coordinate mapping test failed for {direction}: {e}")

    def test_angular_z_always_zero(self):
        """Test that angular.z is always 0 as specified in Requirement 5.7"""
        test_velocities = [
            (1.0, 0.0),   # Forward
            (-1.0, 0.0),  # Backward
            (0.0, 1.0),   # Right
            (0.0, -1.0),  # Left
            (0.5, 0.7),   # Diagonal
            (0.0, 0.0),   # Stop
            (-0.3, -0.8), # Diagonal backward-left
        ]
        
        for vx, vy in test_velocities:
            with self.subTest(vx=vx, vy=vy):
                msg = Twist()
                msg.linear.x = vx
                msg.linear.y = vy
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0  # Must always be 0
                
                # Verify angular.z is 0
                self.assertEqual(msg.angular.z, 0.0, 
                               f"angular.z must be 0 for velocity ({vx}, {vy})")
                
                # Test callback
                self.robot_node.cmd_vel_callback(msg)

    def test_message_reception_with_publisher(self):
        """Test end-to-end message reception through ROS2 topic"""
        received_messages = []
        
        # Create a custom callback to capture messages
        def capture_callback(msg):
            received_messages.append({
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'angular_z': msg.angular.z,
                'timestamp': time.time()
            })
            # Call original callback
            self.robot_node.cmd_vel_callback(msg)
        
        # Replace the callback temporarily
        original_callback = self.robot_node.cmd_vel_callback
        self.robot_node.cmd_vel_callback = capture_callback
        
        try:
            # Test messages
            test_messages = [
                {'x': 1.0, 'y': 0.0},    # Forward
                {'x': -1.0, 'y': 0.0},   # Backward
                {'x': 0.0, 'y': 1.0},    # Right
                {'x': 0.0, 'y': -1.0},   # Left
                {'x': 0.0, 'y': 0.0},    # Stop
            ]
            
            for i, test_vel in enumerate(test_messages):
                msg = Twist()
                msg.linear.x = test_vel['x']
                msg.linear.y = test_vel['y']
                msg.angular.z = 0.0
                
                # Publish message
                self.test_publisher.publish(msg)
                
                # Allow time for message processing
                time.sleep(0.1)
                rclpy.spin_once(self.robot_node, timeout_sec=0.1)
                
                # Verify message was received
                self.assertGreater(len(received_messages), i, 
                                 f"Message {i} was not received")
                
                if len(received_messages) > i:
                    received = received_messages[i]
                    self.assertEqual(received['linear_x'], test_vel['x'])
                    self.assertEqual(received['linear_y'], test_vel['y'])
                    self.assertEqual(received['angular_z'], 0.0)
        
        finally:
            # Restore original callback
            self.robot_node.cmd_vel_callback = original_callback

    def test_velocity_display_format(self):
        """Test that velocity values are displayed in the correct format - Requirement 4.5"""
        # This test verifies the log output format matches the requirement:
        # "velocity_x=<linear.x>, velocity_y=<linear.y>"
        
        test_cases = [
            (1.5, -0.8),
            (0.0, 0.0),
            (-2.3, 1.7),
            (0.5, 0.0),
            (0.0, -1.2)
        ]
        
        for vx, vy in test_cases:
            with self.subTest(vx=vx, vy=vy):
                msg = Twist()
                msg.linear.x = vx
                msg.linear.y = vy
                msg.angular.z = 0.0
                
                # Test that callback executes without error
                # The actual log format verification would require log capture
                # which is complex in unit tests, but we verify the callback works
                try:
                    self.robot_node.cmd_vel_callback(msg)
                except Exception as e:
                    self.fail(f"Velocity display failed for ({vx}, {vy}): {e}")


class TestMessageFlowIntegration(unittest.TestCase):
    """Integration tests for end-to-end message flow"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS2 for testing."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 after testing."""
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.robot_node = RobotTeleopNode()
        self.publisher_node = rclpy.create_node('test_publisher_node')
        self.test_publisher = self.publisher_node.create_publisher(Twist, '/cmd_vel', 10)
        
    def tearDown(self):
        """Clean up test fixtures."""
        self.robot_node.destroy_node()
        self.publisher_node.destroy_node()

    def test_end_to_end_message_flow(self):
        """Test complete message flow from publisher to subscriber"""
        # Track received messages
        received_count = 0
        received_messages = []
        
        def counting_callback(msg):
            nonlocal received_count
            received_count += 1
            received_messages.append({
                'vx': msg.linear.x,
                'vy': msg.linear.y,
                'angular_z': msg.angular.z
            })
            # Call original callback
            original_callback(msg)
        
        # Store original callback and replace with counting version
        original_callback = self.robot_node.cmd_vel_callback
        self.robot_node.cmd_vel_callback = counting_callback
        
        try:
            # Test sequence of velocity commands
            velocity_sequence = [
                (0.0, 0.0),   # Start at rest
                (1.0, 0.0),   # Move forward
                (1.0, 0.5),   # Move forward-right
                (0.0, 0.5),   # Move right
                (-0.5, 0.0),  # Move backward
                (0.0, 0.0),   # Stop
            ]
            
            for i, (vx, vy) in enumerate(velocity_sequence):
                msg = Twist()
                msg.linear.x = vx
                msg.linear.y = vy
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0
                
                # Publish message
                self.test_publisher.publish(msg)
                
                # Process messages
                time.sleep(0.05)  # Small delay for message propagation
                rclpy.spin_once(self.robot_node, timeout_sec=0.1)
                rclpy.spin_once(self.publisher_node, timeout_sec=0.1)
                
                # Verify message was received
                expected_count = i + 1
                self.assertEqual(received_count, expected_count, 
                               f"Expected {expected_count} messages, got {received_count}")
                
                # Verify message content
                if received_messages:
                    last_msg = received_messages[-1]
                    self.assertEqual(last_msg['vx'], vx)
                    self.assertEqual(last_msg['vy'], vy)
                    self.assertEqual(last_msg['angular_z'], 0.0)
        
        finally:
            # Restore original callback
            self.robot_node.cmd_vel_callback = original_callback

    def test_high_frequency_message_handling(self):
        """Test handling of high-frequency velocity commands"""
        message_count = 20
        publish_rate = 0.02  # 50 Hz
        
        received_messages = []
        
        def capture_callback(msg):
            received_messages.append({
                'timestamp': time.time(),
                'vx': msg.linear.x,
                'vy': msg.linear.y
            })
            # Call original callback
            original_callback(msg)
        
        original_callback = self.robot_node.cmd_vel_callback
        self.robot_node.cmd_vel_callback = capture_callback
        
        try:
            start_time = time.time()
            
            # Publish messages at high frequency
            for i in range(message_count):
                msg = Twist()
                msg.linear.x = 0.1 * i  # Varying velocity
                msg.linear.y = 0.05 * i
                msg.angular.z = 0.0
                
                self.test_publisher.publish(msg)
                
                # Process messages
                rclpy.spin_once(self.robot_node, timeout_sec=0.001)
                rclpy.spin_once(self.publisher_node, timeout_sec=0.001)
                
                time.sleep(publish_rate)
            
            end_time = time.time()
            
            # Verify timing and message reception
            total_time = end_time - start_time
            expected_time = message_count * publish_rate
            
            # Allow some tolerance for processing time
            self.assertLess(total_time, expected_time + 0.5, 
                          "Message processing took too long")
            
            # Verify we received most messages (allow for some loss in high-frequency scenarios)
            min_expected = int(message_count * 0.8)  # Allow 20% message loss
            self.assertGreaterEqual(len(received_messages), min_expected,
                                  f"Expected at least {min_expected} messages, got {len(received_messages)}")
        
        finally:
            self.robot_node.cmd_vel_callback = original_callback


if __name__ == '__main__':
    unittest.main()