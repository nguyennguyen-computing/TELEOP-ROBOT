FROM osrf/ros:humble-desktop

# Install additional ROS2 packages and FastDDS
RUN apt-get update && apt-get install -y \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-fastrtps \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Copy robot package
COPY robot/ ./src/teleop_robot/

# Build the workspace with verbose output
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select teleop_robot --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    ls -la install/teleop_robot/lib/teleop_robot/ || echo 'No executables found' && \
    ls -la install/teleop_robot/lib/python3.10/site-packages/teleop_robot/ || echo 'No Python package found'"

# Copy and set up entrypoint
COPY robot/docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

# Copy FastDDS config
COPY fastdds-config.xml /fastdds-config.xml

# Set environment variables for FastDDS
ENV ROS_DOMAIN_ID=0
ENV PYTHONUNBUFFERED=1
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds-config.xml
ENV ROS_LOCALHOST_ONLY=0

ENTRYPOINT ["/docker-entrypoint.sh"]
COPY robot/launch_robot.py /launch_robot.py
RUN chmod +x /launch_robot.py

CMD ["python3", "/launch_robot.py"]
