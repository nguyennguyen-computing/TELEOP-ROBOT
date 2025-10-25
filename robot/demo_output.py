#!/usr/bin/env python3
"""
Demo script showing the expected output format of the robot node.
This simulates what the robot node will output when receiving velocity commands.
"""

import time
from datetime import datetime


def simulate_velocity_commands():
    """Simulate various velocity commands and their expected output."""
    
    print("Robot Teleop Node - Expected Output Demo")
    print("=" * 50)
    print("Format: velocity_x=<x>, velocity_y=<y>")
    print()
    
    # Test cases based on the requirements
    test_cases = [
        # (vx, vy, description)
        (0.0, 0.0, "Stop command - all levels reset to 0"),
        (0.1, 0.0, "Forward movement - up level 1"),
        (0.5, 0.0, "Forward movement - up level 5"),
        (1.0, 0.0, "Forward movement - up level 10 (max)"),
        (-0.3, 0.0, "Backward movement - down level 3"),
        (0.0, 0.2, "Right movement - right level 2"),
        (0.0, -0.4, "Left movement - left level 4"),
        (0.3, 0.2, "Diagonal movement - forward+right"),
        (-0.2, -0.3, "Diagonal movement - backward+left"),
        (0.8, -0.6, "Complex movement - forward+left"),
    ]
    
    for vx, vy, description in test_cases:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[INFO] [{timestamp}] [robot_teleop_node]: velocity_x={vx}, velocity_y={vy}")
        print(f"  → {description}")
        time.sleep(0.5)  # Simulate real-time reception
    
    print()
    print("Demo completed. This shows the expected logging format.")
    print("The actual node will output similar messages when receiving /cmd_vel messages.")


if __name__ == '__main__':
    simulate_velocity_commands()