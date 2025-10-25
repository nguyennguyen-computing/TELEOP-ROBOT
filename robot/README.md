# Teleop Robot ROS2 Package

This ROS2 package contains the robot-side node for the web teleoperation system. The node subscribes to the `/cmd_vel` topic and displays received velocity commands.

## Features

- Subscribes to `/cmd_vel` topic for Twist messages
- Logs velocity values in the format: `velocity_x=<x>, velocity_y=<y>`
- Handles graceful shutdown
- Includes comprehensive error handling
- Docker-ready deployment

## Package Structure

```
robot/
├── teleop_robot/
│   ├── __init__.py
│   └── robot_node.py          # Main ROS2 subscriber node
├── launch/
│   └── robot_teleop.launch.py # Launch file for the node
├── test/
│   └── test_robot_node.py     # Unit tests
├── resource/
│   └── teleop_robot           # Package resource marker
├── package.xml                # ROS2 package manifest
├── setup.py                   # Python package setup
├── Dockerfile                 # Docker container configuration
├── docker-entrypoint.sh       # Docker entrypoint script
└── README.md                  # This file
```

## Usage

### Running with ROS2

1. Build the package:
   ```bash
   colcon build --packages-select teleop_robot
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Run the node:
   ```bash
   ros2 run teleop_robot robot_node
   ```

### Running with Launch File

```bash
ros2 launch teleop_robot robot_teleop.launch.py
```

### Running with Docker

The package is configured for Docker deployment as part of the web teleoperation system:

```bash
docker-compose up robot
```

## Testing

Run the unit tests:

```bash
python3 -m pytest test/test_robot_node.py
```

## Message Format

The node expects `geometry_msgs/Twist` messages on the `/cmd_vel` topic with the following structure:

```
geometry_msgs/Twist:
  linear:
    x: float64    # Forward/backward velocity (m/s)
    y: float64    # Left/right velocity (m/s)  
    z: float64    # Up/down velocity (should be 0.0)
  angular:
    x: float64    # Roll angular velocity (should be 0.0)
    y: float64    # Pitch angular velocity (should be 0.0)
    z: float64    # Yaw angular velocity (should be 0.0)
```

## Coordinate System

- **Forward**: +X direction
- **Backward**: -X direction  
- **Right**: +Y direction
- **Left**: -Y direction

## Logging Output

When velocity commands are received, the node outputs:

```
[INFO] [robot_teleop_node]: velocity_x=1.5, velocity_y=-0.8
```

## Dependencies

- ROS2 Humble
- rclpy
- geometry_msgs
- std_msgs

## Integration

This node is part of the larger web teleoperation system and receives commands through the following flow:

```
Web Interface → Node.js Backend → FastAPI Fleet Server → Zenoh → zenoh-bridge-ros2dds → /cmd_vel → Robot Node
```