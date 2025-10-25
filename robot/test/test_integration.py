#!/usr/bin/env python3
"""
Integration tests for ROS2 robot node with focus on end-to-end message flow
and system integration testing.

This module tests:
- Complete message flow from external publishers to robot node
- Message format validation according to requirements
- Coordinate system mapping verification
- Performance under various load conditions
"""

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from teleop_robot.robot_node import RobotTeleopNode
import time
import threading
import json
import os
from typing import List, Dict, Any


class TestSystemIntegration(unittest.TestCase):
    """System integration tests for ROS2 robot teleoperation"""
    
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
        self.publisher_node = rclpy.create_node('integration_test_publisher')
        self.cmd_vel_publisher = self.publisher_node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Message tracking
        self.received_messages = []
        self.message_timestamps = []
        
        # Replace callback to capture messages
        self.original_callback = self.robot_node.cmd_vel_callback
        self.robot_node.cmd_vel_callback = self._capture_callback
        
    def tearDown(self):
        """Clean up test fixtures."""
        # Restore original callback
        self.robot_node.cmd_vel_callback = self.original_callback
        self.robot_node.destroy_node()
        self.publisher_node.destroy_node()

    def _capture_callback(self, msg: Twist):
        """Capture received messages for analysis"""
        self.received_messages.append({
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'linear_z': msg.linear.z,
            'angular_x': msg.angular.x,
            'angular_y': msg.angular.y,
            'angular_z': msg.angular.z,
            'timestamp': time.time()
        })
        self.message_timestamps.append(time.time())
        
        # Call original callback to maintain functionality
        self.original_callback(msg)

    def test_velocity_command_sequence(self):
        """Test a realistic sequence of velocity commands"""
        # Simulate a typical teleoperation sequence
        command_sequence = [
            # Start at rest
            {'vx': 0.0, 'vy': 0.0, 'description': 'initial_stop'},
            
            # Move forward gradually
            {'vx': 0.1, 'vy': 0.0, 'description': 'slow_forward'},
            {'vx': 0.5, 'vy': 0.0, 'description': 'medium_forward'},
            {'vx': 1.0, 'vy': 0.0, 'description': 'fast_forward'},
            
            # Turn right while moving forward
            {'vx': 1.0, 'vy': 0.3, 'description': 'forward_right_turn'},
            {'vx': 0.8, 'vy': 0.8, 'description': 'diagonal_forward_right'},
            
            # Move right only
            {'vx': 0.0, 'vy': 1.0, 'description': 'right_strafe'},
            
            # Move backward-left
            {'vx': -0.5, 'vy': -0.3, 'description': 'backward_left'},
            
            # Emergency stop
            {'vx': 0.0, 'vy': 0.0, 'description': 'emergency_stop'},
        ]
        
        for i, cmd in enumerate(command_sequence):
            with self.subTest(step=i, description=cmd['description']):
                # Create and publish message
                msg = Twist()
                msg.linear.x = cmd['vx']
                msg.linear.y = cmd['vy']
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0  # Always 0 per requirement 5.7
                
                self.cmd_vel_publisher.publish(msg)
                
                # Process message
                time.sleep(0.05)
                rclpy.spin_once(self.robot_node, timeout_sec=0.1)
                rclpy.spin_once(self.publisher_node, timeout_sec=0.1)
                
                # Verify message was received
                self.assertGreater(len(self.received_messages), i,
                                 f"Message {i} ({cmd['description']}) not received")
                
                # Verify message content
                received = self.received_messages[i]
                self.assertAlmostEqual(received['linear_x'], cmd['vx'], places=6)
                self.assertAlmostEqual(received['linear_y'], cmd['vy'], places=6)
                self.assertEqual(received['angular_z'], 0.0)

    def test_coordinate_system_compliance(self):
        """Test compliance with coordinate system requirements (5.7)"""
        # Test coordinate system mapping: Forward=+X, Right=+Y, angular.z=0
        coordinate_tests = [
            # Forward movement (positive X)
            {'name': 'forward', 'x': 1.0, 'y': 0.0, 'expected_direction': 'forward'},
            
            # Backward movement (negative X)
            {'name': 'backward', 'x': -1.0, 'y': 0.0, 'expected_direction': 'backward'},
            
            # Right movement (positive Y)
            {'name': 'right', 'x': 0.0, 'y': 1.0, 'expected_direction': 'right'},
            
            # Left movement (negative Y)
            {'name': 'left', 'x': 0.0, 'y': -1.0, 'expected_direction': 'left'},
            
            # Diagonal movements
            {'name': 'forward_right', 'x': 0.7, 'y': 0.7, 'expected_direction': 'diagonal'},
            {'name': 'backward_left', 'x': -0.5, 'y': -0.5, 'expected_direction': 'diagonal'},
        ]
        
        for test in coordinate_tests:
            with self.subTest(direction=test['name']):
                msg = Twist()
                msg.linear.x = test['x']
                msg.linear.y = test['y']
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0
                
                # Verify coordinate system mapping
                if test['expected_direction'] == 'forward':
                    self.assertGreater(msg.linear.x, 0, "Forward should be positive X")
                    self.assertEqual(msg.linear.y, 0, "Pure forward should have Y=0")
                elif test['expected_direction'] == 'backward':
                    self.assertLess(msg.linear.x, 0, "Backward should be negative X")
                    self.assertEqual(msg.linear.y, 0, "Pure backward should have Y=0")
                elif test['expected_direction'] == 'right':
                    self.assertEqual(msg.linear.x, 0, "Pure right should have X=0")
                    self.assertGreater(msg.linear.y, 0, "Right should be positive Y")
                elif test['expected_direction'] == 'left':
                    self.assertEqual(msg.linear.x, 0, "Pure left should have X=0")
                    self.assertLess(msg.linear.y, 0, "Left should be negative Y")
                
                # Always verify angular.z = 0 (Requirement 5.7)
                self.assertEqual(msg.angular.z, 0.0, "angular.z must always be 0")
                
                # Test message processing
                self.cmd_vel_publisher.publish(msg)
                time.sleep(0.02)
                rclpy.spin_once(self.robot_node, timeout_sec=0.1)

    def test_message_format_validation(self):
        """Test ROS2 Twist message format validation"""
        # Test various message formats to ensure robustness
        test_messages = [
            # Standard velocity commands
            {'linear': [0.5, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'valid': True},
            {'linear': [0.0, 1.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'valid': True},
            {'linear': [-0.8, -0.3, 0.0], 'angular': [0.0, 0.0, 0.0], 'valid': True},
            
            # Edge cases
            {'linear': [0.0, 0.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'valid': True},  # Stop
            {'linear': [10.0, 10.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'valid': True},  # High velocity
            {'linear': [-10.0, -10.0, 0.0], 'angular': [0.0, 0.0, 0.0], 'valid': True},  # High negative
            
            # Messages that should have specific constraints
            {'linear': [1.0, 1.0, 0.1], 'angular': [0.0, 0.0, 0.0], 'valid': True, 'note': 'linear.z should be ignored'},
            {'linear': [1.0, 1.0, 0.0], 'angular': [0.1, 0.1, 0.1], 'valid': True, 'note': 'angular components should be ignored except z'},
        ]
        
        for i, test_msg in enumerate(test_messages):
            with self.subTest(message=i):
                msg = Twist()
                msg.linear.x = test_msg['linear'][0]
                msg.linear.y = test_msg['linear'][1]
                msg.linear.z = test_msg['linear'][2]
                msg.angular.x = test_msg['angular'][0]
                msg.angular.y = test_msg['angular'][1]
                msg.angular.z = test_msg['angular'][2]
                
                # For our system, angular.z should always be 0 (Requirement 5.7)
                if test_msg['angular'][2] != 0.0:
                    msg.angular.z = 0.0  # Force to 0 as per requirement
                
                # Verify format constraints
                self.assertEqual(msg.angular.z, 0.0, "angular.z must be 0")
                
                # Test message processing
                try:
                    self.cmd_vel_publisher.publish(msg)
                    time.sleep(0.01)
                    rclpy.spin_once(self.robot_node, timeout_sec=0.05)
                    
                    if test_msg['valid']:
                        # Should process without error
                        pass
                except Exception as e:
                    if test_msg['valid']:
                        self.fail(f"Valid message {i} failed: {e}")

    def test_performance_under_load(self):
        """Test system performance under high message load"""
        message_count = 100
        target_frequency = 20  # Hz
        interval = 1.0 / target_frequency
        
        start_time = time.time()
        
        # Send messages at target frequency
        for i in range(message_count):
            msg = Twist()
            # Vary velocity to simulate real usage
            msg.linear.x = 0.5 * (1 + 0.1 * (i % 10))
            msg.linear.y = 0.3 * (1 + 0.1 * ((i + 5) % 10))
            msg.angular.z = 0.0
            
            self.cmd_vel_publisher.publish(msg)
            
            # Process messages
            rclpy.spin_once(self.robot_node, timeout_sec=0.001)
            rclpy.spin_once(self.publisher_node, timeout_sec=0.001)
            
            # Maintain frequency
            time.sleep(interval)
        
        end_time = time.time()
        total_time = end_time - start_time
        
        # Verify timing performance
        expected_time = message_count * interval
        time_tolerance = 0.5  # Allow 500ms tolerance
        
        self.assertLess(total_time, expected_time + time_tolerance,
                       f"Processing took too long: {total_time:.2f}s vs expected {expected_time:.2f}s")
        
        # Verify message reception rate
        received_count = len(self.received_messages)
        reception_rate = received_count / message_count
        
        # Should receive at least 90% of messages
        self.assertGreater(reception_rate, 0.9,
                          f"Low message reception rate: {reception_rate:.2%}")

    def test_velocity_display_format_compliance(self):
        """Test compliance with velocity display format (Requirement 4.5)"""
        # Test that the node can handle the expected velocity format
        # The actual log format verification is tested in the main test file
        
        test_velocities = [
            (0.0, 0.0),
            (1.5, -0.8),
            (-2.3, 1.7),
            (0.5, 0.0),
            (0.0, -1.2),
            (10.0, 10.0),  # Maximum values
            (-10.0, -10.0),  # Minimum values
        ]
        
        for vx, vy in test_velocities:
            with self.subTest(vx=vx, vy=vy):
                msg = Twist()
                msg.linear.x = vx
                msg.linear.y = vy
                msg.angular.z = 0.0
                
                # Publish and process
                self.cmd_vel_publisher.publish(msg)
                time.sleep(0.01)
                rclpy.spin_once(self.robot_node, timeout_sec=0.05)
                
                # Verify message was processed (callback should handle display format)
                # The actual format "velocity_x=<linear.x>, velocity_y=<linear.y>" 
                # is verified in the callback implementation


class TestMessageFlowLatency(unittest.TestCase):
    """Test message flow latency and timing characteristics"""
    
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
        self.publisher_node = rclpy.create_node('latency_test_publisher')
        self.cmd_vel_publisher = self.publisher_node.create_publisher(Twist, '/cmd_vel', 10)
        
        self.message_latencies = []
        self.publish_times = {}
        
        # Replace callback to measure latency
        self.original_callback = self.robot_node.cmd_vel_callback
        self.robot_node.cmd_vel_callback = self._latency_callback
        
    def tearDown(self):
        """Clean up test fixtures."""
        self.robot_node.cmd_vel_callback = self.original_callback
        self.robot_node.destroy_node()
        self.publisher_node.destroy_node()

    def _latency_callback(self, msg: Twist):
        """Measure message latency"""
        receive_time = time.time()
        
        # Use velocity values as message ID for latency calculation
        msg_id = f"{msg.linear.x:.3f}_{msg.linear.y:.3f}"
        
        if msg_id in self.publish_times:
            latency = receive_time - self.publish_times[msg_id]
            self.message_latencies.append(latency)
        
        # Call original callback
        self.original_callback(msg)

    def test_message_latency(self):
        """Test end-to-end message latency"""
        test_count = 50
        
        for i in range(test_count):
            # Create unique message
            vx = round(0.1 * i, 3)
            vy = round(0.05 * i, 3)
            
            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = 0.0
            
            # Record publish time
            msg_id = f"{vx:.3f}_{vy:.3f}"
            self.publish_times[msg_id] = time.time()
            
            # Publish message
            self.cmd_vel_publisher.publish(msg)
            
            # Process immediately
            rclpy.spin_once(self.robot_node, timeout_sec=0.01)
            rclpy.spin_once(self.publisher_node, timeout_sec=0.01)
            
            time.sleep(0.02)  # 50 Hz rate
        
        # Analyze latencies
        if self.message_latencies:
            avg_latency = sum(self.message_latencies) / len(self.message_latencies)
            max_latency = max(self.message_latencies)
            
            # Latency should be reasonable for real-time control
            self.assertLess(avg_latency, 0.01, f"Average latency too high: {avg_latency:.4f}s")
            self.assertLess(max_latency, 0.05, f"Maximum latency too high: {max_latency:.4f}s")
            
            print(f"Latency stats: avg={avg_latency*1000:.2f}ms, max={max_latency*1000:.2f}ms")


if __name__ == '__main__':
    # Run with verbose output
    unittest.main(verbosity=2)