#!/bin/bash
# Development Setup Script
# Sets up the development environment with proper .env files

echo "🚀 TELEOP ROBOT SYSTEM - DEVELOPMENT SETUP"
echo "=========================================="

# Check if root .env exists
if [ ! -f ".env" ]; then
    echo "❌ Root .env file not found!"
    echo "💡 Please copy .env.example to .env and configure it"
    exit 1
fi

echo "✅ Root .env file found"

# Sync backend and frontend environment
echo "🔄 Syncing backend and frontend environment variables..."
python3 sync-frontend-env.py

if [ $? -eq 0 ]; then
    echo "✅ Backend and frontend environment synced successfully"
else
    echo "❌ Failed to sync backend and frontend environment"
    exit 1
fi

# Validate all environment variables
echo "🔍 Validating environment configuration..."
python3 validate-env.py

if [ $? -eq 0 ]; then
    echo "✅ Environment validation passed"
else
    echo "❌ Environment validation failed"
    exit 1
fi

echo ""
echo "🎉 DEVELOPMENT SETUP COMPLETE!"
echo "================================"
echo "✅ Root .env configured"
echo "✅ Backend and frontend .env files synced"
echo "✅ All environment variables validated"
echo ""
echo "🚀 You can now start the system:"
echo "   docker-compose up -d"
echo ""
echo "🌐 Or run services locally:"
echo "   Backend: cd backend && npm run dev"
echo "   Frontend: cd frontend && npm run dev"