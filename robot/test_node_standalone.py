#!/usr/bin/env python3
"""
Standalone test script to verify the robot node functionality.
This can be run without Docker to test the node implementation.
"""

import sys
import os
import time
import threading
from geometry_msgs.msg import Twist

# Add the teleop_robot package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

try:
    import rclpy
    from teleop_robot.robot_node import RobotTeleopNode
    ROS2_AVAILABLE = True
except ImportError:
    print("ROS2 not available. This test requires ROS2 to be installed.")
    ROS2_AVAILABLE = False


def test_node_creation():
    """Test that the node can be created successfully."""
    if not ROS2_AVAILABLE:
        print("SKIP: ROS2 not available")
        return False
    
    try:
        rclpy.init()
        node = RobotTeleopNode()
        print("✓ Node created successfully")
        
        # Check node properties
        assert node.get_name() == 'robot_teleop_node'
        print("✓ Node name is correct")
        
        assert node.subscription is not None
        print("✓ Subscription created successfully")
        
        node.destroy_node()
        rclpy.shutdown()
        return True
        
    except Exception as e:
        print(f"✗ Node creation failed: {e}")
        return False


def test_callback_function():
    """Test the callback function directly."""
    if not ROS2_AVAILABLE:
        print("SKIP: ROS2 not available")
        return False
    
    try:
        rclpy.init()
        node = RobotTeleopNode()
        
        # Create test message
        test_msg = Twist()
        test_msg.linear.x = 1.5
        test_msg.linear.y = -0.8
        test_msg.linear.z = 0.0
        test_msg.angular.x = 0.0
        test_msg.angular.y = 0.0
        test_msg.angular.z = 0.0
        
        # Call callback directly
        node.cmd_vel_callback(test_msg)
        print("✓ Callback function executed successfully")
        
        node.destroy_node()
        rclpy.shutdown()
        return True
        
    except Exception as e:
        print(f"✗ Callback test failed: {e}")
        return False


def main():
    """Run all tests."""
    print("Testing Robot Teleop Node Implementation")
    print("=" * 50)
    
    tests = [
        ("Node Creation", test_node_creation),
        ("Callback Function", test_callback_function),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\nRunning: {test_name}")
        if test_func():
            passed += 1
        else:
            print(f"Failed: {test_name}")
    
    print(f"\n" + "=" * 50)
    print(f"Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ All tests passed!")
        return 0
    else:
        print("✗ Some tests failed!")
        return 1


if __name__ == '__main__':
    sys.exit(main())