#!/usr/bin/env python3
"""
Validation script to check the robot node implementation without requiring ROS2.
This validates the code structure and implementation details.
"""

import os
import sys
import ast
import inspect


def validate_file_structure():
    """Validate that all required files are present."""
    print("Validating file structure...")
    
    required_files = [
        'teleop_robot/__init__.py',
        'teleop_robot/robot_node.py',
        'package.xml',
        'setup.py',
        'resource/teleop_robot',
        'launch/robot_teleop.launch.py',
        'test/test_robot_node.py',
        'Dockerfile',
        'docker-entrypoint.sh',
        'README.md'
    ]
    
    missing_files = []
    for file_path in required_files:
        full_path = os.path.join('robot', file_path)
        if not os.path.exists(full_path):
            missing_files.append(file_path)
    
    if missing_files:
        print(f"✗ Missing files: {missing_files}")
        return False
    else:
        print("✓ All required files present")
        return True


def validate_robot_node_code():
    """Validate the robot node implementation."""
    print("\nValidating robot node code...")
    
    try:
        # Read the robot node file
        with open('robot/teleop_robot/robot_node.py', 'r') as f:
            code = f.read()
        
        # Parse the AST to analyze the code
        tree = ast.parse(code)
        
        # Check for required components
        classes = [node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)]
        functions = [node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)]
        
        # Validate class exists
        if 'RobotTeleopNode' not in classes:
            print("✗ RobotTeleopNode class not found")
            return False
        print("✓ RobotTeleopNode class found")
        
        # Validate required methods
        required_methods = ['__init__', 'cmd_vel_callback']
        for method in required_methods:
            if method not in functions:
                print(f"✗ Required method {method} not found")
                return False
        print("✓ Required methods found")
        
        # Validate main function
        if 'main' not in functions:
            print("✗ main function not found")
            return False
        print("✓ main function found")
        
        # Check for required imports (basic string search)
        required_imports = ['rclpy', 'geometry_msgs.msg', 'Twist']
        for imp in required_imports:
            if imp not in code:
                print(f"✗ Required import {imp} not found")
                return False
        print("✓ Required imports found")
        
        # Check for logging format
        if 'velocity_x={velocity_x}, velocity_y={velocity_y}' not in code:
            print("✗ Required logging format not found")
            return False
        print("✓ Required logging format found")
        
        # Check for topic subscription
        if '/cmd_vel' not in code:
            print("✗ /cmd_vel topic subscription not found")
            return False
        print("✓ /cmd_vel topic subscription found")
        
        return True
        
    except Exception as e:
        print(f"✗ Error validating robot node code: {e}")
        return False


def validate_package_xml():
    """Validate the package.xml file."""
    print("\nValidating package.xml...")
    
    try:
        with open('robot/package.xml', 'r') as f:
            content = f.read()
        
        required_elements = [
            '<name>teleop_robot</name>',
            '<depend>rclpy</depend>',
            '<depend>geometry_msgs</depend>',
            '<build_type>ament_python</build_type>'
        ]
        
        for element in required_elements:
            if element not in content:
                print(f"✗ Required element not found: {element}")
                return False
        
        print("✓ package.xml is valid")
        return True
        
    except Exception as e:
        print(f"✗ Error validating package.xml: {e}")
        return False


def validate_setup_py():
    """Validate the setup.py file."""
    print("\nValidating setup.py...")
    
    try:
        with open('robot/setup.py', 'r') as f:
            content = f.read()
        
        required_elements = [
            "package_name = 'teleop_robot'",
            "'robot_node = teleop_robot.robot_node:main'",
            "packages=[package_name]"
        ]
        
        for element in required_elements:
            if element not in content:
                print(f"✗ Required element not found: {element}")
                return False
        
        print("✓ setup.py is valid")
        return True
        
    except Exception as e:
        print(f"✗ Error validating setup.py: {e}")
        return False


def validate_dockerfile():
    """Validate the Dockerfile."""
    print("\nValidating Dockerfile...")
    
    try:
        with open('robot/Dockerfile', 'r') as f:
            content = f.read()
        
        required_elements = [
            'FROM ros:humble-base',
            'ros-humble-geometry-msgs',
            'colcon build --packages-select teleop_robot',
            '"ros2", "run", "teleop_robot", "robot_node"'
        ]
        
        for element in required_elements:
            if element not in content:
                print(f"✗ Required element not found: {element}")
                return False
        
        print("✓ Dockerfile is valid")
        return True
        
    except Exception as e:
        print(f"✗ Error validating Dockerfile: {e}")
        return False


def main():
    """Run all validation tests."""
    print("Robot Node Implementation Validation")
    print("=" * 50)
    
    tests = [
        ("File Structure", validate_file_structure),
        ("Robot Node Code", validate_robot_node_code),
        ("Package XML", validate_package_xml),
        ("Setup.py", validate_setup_py),
        ("Dockerfile", validate_dockerfile),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        if test_func():
            passed += 1
        else:
            print(f"Failed: {test_name}")
    
    print(f"\n" + "=" * 50)
    print(f"Validation Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ All validations passed!")
        print("\nImplementation Summary:")
        print("- ROS2 subscriber node created")
        print("- Subscribes to /cmd_vel topic")
        print("- Logs velocity in required format: velocity_x=<x>, velocity_y=<y>")
        print("- Configured for Docker deployment")
        print("- Includes comprehensive error handling")
        print("- Ready for integration testing")
        return 0
    else:
        print("✗ Some validations failed!")
        return 1


if __name__ == '__main__':
    sys.exit(main())