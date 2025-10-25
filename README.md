# Web Teleop Robot System

A web-based teleoperation system for robots using Next.js, Node.js, FastAPI, and Zenoh.

## 🏗️ System Overview

![System Overview](docs/system-overview.jpeg)

**Services:**
- **Frontend**: Web control interface (Next.js)
- **Backend API**: REST API and WebSocket (Node.js)
- **Fleet Server**: Robot communication (FastAPI + Zenoh)
- **Robot**: Simulated robot for testing (ROS2)
- **MongoDB**: Data persistence
- **Zenoh Bridge**: ROS2 integration

## 🚀 Quick Start

### Prerequisites
- Docker and Docker Compose
- Git

### Setup and Run
```bash
# 1. Clone and setup
git clone <repository-url>
cd teleop-robot
cp .env.example .env
chmod +x docker-cleanup-and-run.sh

# 2. Start all services (recommended)
./docker-cleanup-and-run.sh

# 3. Access the application
# Web Interface: http://localhost:3000
# Backend API: http://localhost:3001
```

### Alternative Commands
```bash
# Manual start (all services)
docker-compose up -d --build

# Start with robot simulation
docker-compose --profile robot up -d --build

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

## ⚙️ Configuration

Key environment variables in `.env`:
```bash
# Ports
WEB_PORT=3000
NODE_PORT=3001
FLEET_PORT=8000

# Robot Control
VX_MAX=2.0
VY_MAX=2.0

# Authentication
AUTH_ENABLED=true
API_KEYS=test-api-key-123,admin-key-456
JWT_SECRET=your-secret-key-here
```

## 📡 API Endpoints

### Authentication
```bash
# Get JWT token
POST /api/auth/token
{
  "userId": "test-user",
  "scopes": ["robot:control"]
}

# Use API key
Authorization: ApiKey test-api-key-123

# Use JWT token
Authorization: Bearer <jwt-token>
```

### Robot Control
```bash
# Send velocity command
POST /api/vel
{
  "vx": 0.2,
  "vy": 0.1,
  "levels": {
    "up": 2,
    "down": 0,
    "left": 0,
    "right": 1
  }
}
```

### Health Checks
- `GET /health` - Backend health
- `GET /api/health` - API health
- `GET /status` - Fleet server status (port 8000)

## 🐛 Troubleshooting

### Common Issues
```bash
# Port conflicts
./docker-cleanup-and-run.sh

# Rebuild after config changes
docker-compose build --no-cache
docker-compose up -d

# Check service status
docker-compose ps
docker-compose logs -f <service-name>

# Clean everything
docker-compose down -v
docker system prune -a
```

### Service URLs
- **Web Interface**: http://localhost:3000
- **Backend API**: http://localhost:3001/health
- **Fleet API**: http://localhost:8000/status
- **MongoDB**: localhost:27017

## 🛠️ Development

### Local Development
```bash
# Install dependencies
npm install
npm run install:all

# Start in development mode
npm run dev
```

### Docker Development
```bash
# Restart specific service
docker-compose restart web

# Rebuild specific service
docker-compose build web --no-cache

# Start with robot simulation
docker-compose --profile robot up -d

# Execute commands in container
docker-compose exec web sh
docker-compose exec node-api npm test
docker-compose exec robot bash
```

## 📊 Monitoring

```bash
# View all logs
docker-compose logs -f

# View specific service logs
docker-compose logs -f web
docker-compose logs -f node-api

# Check resource usage
docker stats

# Service health
curl http://localhost:3001/health
curl http://localhost:8000/heatlth
```

## 📄 License

MIT License - see LICENSE file for details.