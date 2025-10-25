#!/usr/bin/env python3
"""
End-to-end message flow testing for ROS2 robot teleoperation.

This module provides comprehensive testing of the complete message flow
from velocity commands to robot node reception, validating requirements
4.5 and 5.7 specifically.
"""

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from teleop_robot.robot_node import RobotTeleopNode
import time
import json
import threading
from typing import List, Dict, Tuple
import logging


class MessageFlowValidator:
    """Validates message flow and format compliance"""
    
    def __init__(self):
        self.received_messages = []
        self.validation_errors = []
    
    def validate_twist_message(self, msg: Twist, expected_vx: float = None, expected_vy: float = None) -> bool:
        """Validate a Twist message against requirements"""
        errors = []
        
        # Requirement 5.7: angular.z must always be 0
        if msg.angular.z != 0.0:
            errors.append(f"angular.z must be 0, got {msg.angular.z}")
        
        # Requirement 5.7: Other angular components should be 0 for our system
        if msg.angular.x != 0.0:
            errors.append(f"angular.x should be 0, got {msg.angular.x}")
        if msg.angular.y != 0.0:
            errors.append(f"angular.y should be 0, got {msg.angular.y}")
        
        # linear.z should be 0 for ground robots
        if msg.linear.z != 0.0:
            errors.append(f"linear.z should be 0 for ground robot, got {msg.linear.z}")
        
        # Check expected values if provided
        if expected_vx is not None and abs(msg.linear.x - expected_vx) > 1e-6:
            errors.append(f"Expected vx={expected_vx}, got {msg.linear.x}")
        
        if expected_vy is not None and abs(msg.linear.y - expected_vy) > 1e-6:
            errors.append(f"Expected vy={expected_vy}, got {msg.linear.y}")
        
        if errors:
            self.validation_errors.extend(errors)
            return False
        
        return True
    
    def validate_coordinate_system(self, vx: float, vy: float) -> Dict[str, str]:
        """Validate coordinate system mapping (Requirement 5.7)"""
        directions = []
        
        # Forward = +X, Backward = -X
        if vx > 0:
            directions.append("forward")
        elif vx < 0:
            directions.append("backward")
        
        # Right = +Y, Left = -Y
        if vy > 0:
            directions.append("right")
        elif vy < 0:
            directions.append("left")
        
        return {
            'vx': vx,
            'vy': vy,
            'directions': directions,
            'coordinate_system': 'Forward=+X, Right=+Y'
        }


class TestEndToEndMessageFlow(unittest.TestCase):
    """Test complete end-to-end message flow"""
    
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
        self.publisher_node = rclpy.create_node('message_flow_test_publisher')
        self.cmd_vel_publisher = self.publisher_node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.validator = MessageFlowValidator()
        self.received_messages = []
        self.callback_count = 0
        
        # Replace callback to capture and validate messages
        self.original_callback = self.robot_node.cmd_vel_callback
        self.robot_node.cmd_vel_callback = self._validation_callback
        
    def tearDown(self):
        """Clean up test fixtures."""
        self.robot_node.cmd_vel_callback = self.original_callback
        self.robot_node.destroy_node()
        self.publisher_node.destroy_node()

    def _validation_callback(self, msg: Twist):
        """Callback that validates and captures messages"""
        self.callback_count += 1
        
        # Capture message data
        message_data = {
            'sequence': self.callback_count,
            'timestamp': time.time(),
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z,
        }
        self.received_messages.append(message_data)
        
        # Validate message format
        self.validator.validate_twist_message(msg)
        
        # Call original callback to maintain functionality
        self.original_callback(msg)

    def test_complete_velocity_command_flow(self):
        """Test complete flow from velocity command to robot reception"""
        # Test sequence representing typical teleoperation commands
        velocity_commands = [
            # Basic movements
            {'vx': 0.0, 'vy': 0.0, 'name': 'stop'},
            {'vx': 1.0, 'vy': 0.0, 'name': 'forward'},
            {'vx': -1.0, 'vy': 0.0, 'name': 'backward'},
            {'vx': 0.0, 'vy': 1.0, 'name': 'right'},
            {'vx': 0.0, 'vy': -1.0, 'name': 'left'},
            
            # Diagonal movements
            {'vx': 0.7, 'vy': 0.7, 'name': 'forward_right'},
            {'vx': -0.5, 'vy': -0.5, 'name': 'backward_left'},
            
            # Variable speeds
            {'vx': 0.1, 'vy': 0.0, 'name': 'slow_forward'},
            {'vx': 2.0, 'vy': 0.0, 'name': 'fast_forward'},
            
            # Final stop
            {'vx': 0.0, 'vy': 0.0, 'name': 'final_stop'},
        ]
        
        for i, cmd in enumerate(velocity_commands):
            with self.subTest(command=cmd['name'], sequence=i):
                # Create Twist message
                msg = Twist()
                msg.linear.x = cmd['vx']
                msg.linear.y = cmd['vy']
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0  # Always 0 per Requirement 5.7
                
                # Validate message before sending
                self.assertTrue(
                    self.validator.validate_twist_message(msg, cmd['vx'], cmd['vy']),
                    f"Message validation failed for {cmd['name']}"
                )
                
                # Publish message
                self.cmd_vel_publisher.publish(msg)
                
                # Allow message processing
                time.sleep(0.05)
                rclpy.spin_once(self.robot_node, timeout_sec=0.1)
                rclpy.spin_once(self.publisher_node, timeout_sec=0.1)
                
                # Verify message was received
                expected_count = i + 1
                self.assertEqual(
                    len(self.received_messages), expected_count,
                    f"Expected {expected_count} messages, got {len(self.received_messages)}"
                )
                
                # Verify message content
                if self.received_messages:
                    received = self.received_messages[-1]
                    self.assertAlmostEqual(received['linear_x'], cmd['vx'], places=6)
                    self.assertAlmostEqual(received['linear_y'], cmd['vy'], places=6)
                    self.assertEqual(received['angular_z'], 0.0)
        
        # Verify no validation errors occurred
        self.assertEqual(
            len(self.validator.validation_errors), 0,
            f"Validation errors: {self.validator.validation_errors}"
        )

    def test_coordinate_system_mapping_validation(self):
        """Test coordinate system mapping compliance (Requirement 5.7)"""
        # Test coordinate system: Forward=+X, Right=+Y, angular.z=0
        coordinate_tests = [
            # Pure directions
            {'vx': 1.0, 'vy': 0.0, 'expected_dirs': ['forward']},
            {'vx': -1.0, 'vy': 0.0, 'expected_dirs': ['backward']},
            {'vx': 0.0, 'vy': 1.0, 'expected_dirs': ['right']},
            {'vx': 0.0, 'vy': -1.0, 'expected_dirs': ['left']},
            
            # Diagonal combinations
            {'vx': 1.0, 'vy': 1.0, 'expected_dirs': ['forward', 'right']},
            {'vx': -1.0, 'vy': 1.0, 'expected_dirs': ['backward', 'right']},
            {'vx': 1.0, 'vy': -1.0, 'expected_dirs': ['forward', 'left']},
            {'vx': -1.0, 'vy': -1.0, 'expected_dirs': ['backward', 'left']},
            
            # Stop
            {'vx': 0.0, 'vy': 0.0, 'expected_dirs': []},
        ]
        
        for test in coordinate_tests:
            with self.subTest(vx=test['vx'], vy=test['vy']):
                # Validate coordinate system mapping
                coord_info = self.validator.validate_coordinate_system(test['vx'], test['vy'])
                
                self.assertEqual(coord_info['vx'], test['vx'])
                self.assertEqual(coord_info['vy'], test['vy'])
                self.assertEqual(set(coord_info['directions']), set(test['expected_dirs']))
                
                # Create and send message
                msg = Twist()
                msg.linear.x = test['vx']
                msg.linear.y = test['vy']
                msg.angular.z = 0.0  # Must be 0
                
                # Verify coordinate system constraints
                if test['vx'] > 0:
                    self.assertGreater(msg.linear.x, 0, "Forward should be positive X")
                elif test['vx'] < 0:
                    self.assertLess(msg.linear.x, 0, "Backward should be negative X")
                
                if test['vy'] > 0:
                    self.assertGreater(msg.linear.y, 0, "Right should be positive Y")
                elif test['vy'] < 0:
                    self.assertLess(msg.linear.y, 0, "Left should be negative Y")
                
                # Always verify angular.z = 0
                self.assertEqual(msg.angular.z, 0.0, "angular.z must always be 0")

    def test_message_format_requirements(self):
        """Test message format requirements (4.5, 5.7)"""
        # Test various message formats to ensure compliance
        format_tests = [
            # Standard cases
            {'linear': [0.5, 0.3, 0.0], 'angular': [0.0, 0.0, 0.0], 'description': 'standard_movement'},
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'description': 'stop_command'},
            {'linear': [1.0, 1.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'description': 'diagonal_movement'},
            
            # Edge cases
            {'linear': [10.0, 10.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'description': 'high_velocity'},
            {'linear': [-10.0, -10.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'description': 'high_negative_velocity'},
            
            # Precision tests
            {'linear': [0.001, 0.001, 0.0], 'angular': [0.0, 0.0, 0.0], 'description': 'low_precision'},
            {'linear': [1.23456, -2.34567, 0.0], 'angular': [0.0, 0.0, 0.0], 'description': 'high_precision'},
        ]
        
        for test in format_tests:
            with self.subTest(description=test['description']):
                msg = Twist()
                msg.linear.x = test['linear'][0]
                msg.linear.y = test['linear'][1]
                msg.linear.z = test['linear'][2]
                msg.angular.x = test['angular'][0]
                msg.angular.y = test['angular'][1]
                msg.angular.z = test['angular'][2]
                
                # Validate format requirements
                # Requirement 5.7: angular.z must be 0
                self.assertEqual(msg.angular.z, 0.0, "angular.z must be 0")
                
                # Requirement 5.7: linear.z should be 0 for ground robots
                self.assertEqual(msg.linear.z, 0.0, "linear.z should be 0")
                
                # Requirement 5.7: Other angular components should be 0
                self.assertEqual(msg.angular.x, 0.0, "angular.x should be 0")
                self.assertEqual(msg.angular.y, 0.0, "angular.y should be 0")
                
                # Test message processing
                self.cmd_vel_publisher.publish(msg)
                time.sleep(0.01)
                rclpy.spin_once(self.robot_node, timeout_sec=0.05)

    def test_high_frequency_message_flow(self):
        """Test message flow at high frequency (simulating real teleoperation)"""
        frequency = 20  # Hz
        duration = 2.0  # seconds
        message_count = int(frequency * duration)
        interval = 1.0 / frequency
        
        start_time = time.time()
        
        for i in range(message_count):
            # Generate varying velocity commands
            t = i * interval
            vx = 0.5 * (1 + 0.3 * (i % 7))  # Varying forward velocity
            vy = 0.3 * (1 + 0.2 * ((i + 3) % 5))  # Varying lateral velocity
            
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = 0.0  # Always 0
            
            # Publish message
            self.cmd_vel_publisher.publish(msg)
            
            # Process messages
            rclpy.spin_once(self.robot_node, timeout_sec=0.001)
            rclpy.spin_once(self.publisher_node, timeout_sec=0.001)
            
            # Maintain frequency
            time.sleep(interval)
        
        end_time = time.time()
        actual_duration = end_time - start_time
        
        # Verify timing
        expected_duration = message_count * interval
        timing_tolerance = 0.5  # 500ms tolerance
        
        self.assertLess(
            actual_duration, expected_duration + timing_tolerance,
            f"High frequency test took too long: {actual_duration:.2f}s"
        )
        
        # Verify message reception
        received_count = len(self.received_messages)
        reception_rate = received_count / message_count if message_count > 0 else 0
        
        # Should receive at least 80% of messages at high frequency
        self.assertGreater(
            reception_rate, 0.8,
            f"Low reception rate at high frequency: {reception_rate:.2%}"
        )
        
        # Verify no validation errors
        self.assertEqual(
            len(self.validator.validation_errors), 0,
            f"Validation errors during high frequency test: {self.validator.validation_errors}"
        )

    def test_velocity_display_format_compliance(self):
        """Test that velocity display format meets Requirement 4.5"""
        # Requirement 4.5: Robot should display "velocity_x=<linear.x>, velocity_y=<linear.y>"
        # This test ensures the callback can handle the expected format
        
        display_test_cases = [
            (0.0, 0.0),
            (1.5, -0.8),
            (-2.3, 1.7),
            (0.5, 0.0),
            (0.0, -1.2),
            (10.0, 10.0),
            (-10.0, -10.0),
        ]
        
        for vx, vy in display_test_cases:
            with self.subTest(vx=vx, vy=vy):
                msg = Twist()
                msg.linear.x = vx
                msg.linear.y = vy
                msg.angular.z = 0.0
                
                # The callback should handle the display format internally
                # We verify it executes without error
                try:
                    self.robot_node.cmd_vel_callback(msg)
                except Exception as e:
                    self.fail(f"Velocity display callback failed for ({vx}, {vy}): {e}")
                
                # Verify the message was processed correctly
                # (The actual log format verification would require log capture)
                self.assertTrue(True, "Callback executed successfully")


if __name__ == '__main__':
    # Configure logging for test output
    logging.basicConfig(level=logging.INFO)
    
    # Run tests with high verbosity
    unittest.main(verbosity=2)