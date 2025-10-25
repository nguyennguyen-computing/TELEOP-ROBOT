#!/bin/bash

# Clean up and run Docker services

set -e

echo "🧹 Cleaning up existing containers and ports..."

# Stop all containers
docker-compose -f docker-compose.yml down --remove-orphans

# Clean up Docker build cache
echo "🗑️  Cleaning Docker build cache..."
docker builder prune -f

# Kill any processes using our ports
echo "🔌 Freeing up ports..."
lsof -ti:3000,3001,8000,7447,27017 | xargs kill -9 2>/dev/null || true

# Wait a moment for ports to be freed
sleep 2

echo "🚀 Starting backend services..."

# Start services in order
docker-compose -f docker-compose.yml up -d mongo
echo "⏳ Waiting for MongoDB..."
sleep 10

docker-compose -f docker-compose.yml up -d zenoh
echo "⏳ Waiting for Zenoh..."
sleep 5

docker-compose -f docker-compose.yml up -d bridge
echo "⏳ Waiting for bridge..."
sleep 5

docker-compose -f docker-compose.yml up -d robot
echo "⏳ Waiting for robot..."
sleep 5

docker-compose -f docker-compose.yml up -d --build fleet-api
echo "⏳ Waiting for Fleet API..."
sleep 10

docker-compose -f docker-compose.yml up -d --build node-api
echo "⏳ Waiting for Node API..."
sleep 10

echo "🏥 Checking service status..."
docker-compose -f docker-compose.yml ps

echo "🎉 Backend services are ready!"

docker-compose -f docker-compose.yml up -d --build web
echo "⏳ Waiting for Node API..."
sleep 10