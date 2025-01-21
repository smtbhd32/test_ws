# .github/actions/colcon-build-action/Dockerfile
FROM ros:humble

# Install dependencies and colcon
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace environment
WORKDIR /workspace

# Install ROS setup script for ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command (could be overridden)
CMD ["bash"]
