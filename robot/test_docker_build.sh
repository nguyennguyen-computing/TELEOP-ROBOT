#!/bin/bash

# Test script to validate Docker build for the robot node
echo "Testing Robot Node Docker Build"
echo "================================"

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed or not in PATH"
    exit 1
fi

echo "✅ Docker is available"

# Build the robot image
echo "🔨 Building robot Docker image..."
if docker build -t teleop-robot:test robot/; then
    echo "✅ Docker build successful"
else
    echo "❌ Docker build failed"
    exit 1
fi

# Test that the image was created
if docker images | grep -q "teleop-robot.*test"; then
    echo "✅ Docker image created successfully"
else
    echo "❌ Docker image not found"
    exit 1
fi

# Clean up test image
echo "🧹 Cleaning up test image..."
docker rmi teleop-robot:test

echo "✅ All Docker tests passed!"
echo ""
echo "The robot node is ready for deployment with:"
echo "  docker-compose up robot"