# Teleop Robot Backend API

## 🚀 Quick Start

### Prerequisites
- Node.js (v18+)
- MongoDB
- Docker (optional, for MongoDB)

### Running the Services

#### 1. Start MongoDB
```bash
# Using Docker (recommended)
docker run -d --name teleop-mongo -p 27017:27017 -e MONGO_INITDB_DATABASE=teleop_db mongo:7

# Or using local MongoDB installation
mongod --dbpath /path/to/your/db
```

#### 2. Start Backend API
```bash
cd backend
npm install
npm run build

# Set environment variables and start
NODE_PORT=3001 \
FLEET_API_URL=http://localhost:8000 \
MONGO_URI=mongodb://localhost:27017/teleop_db \
CORS_ORIGIN=http://localhost:3000 \
VX_MAX=1.0 \
VY_MAX=1.0 \
ZENOH_LOCATOR=tcp/localhost:7447 \
Z_KEY_CMD_VEL=rt/ros2/cmd_vel \
JWT_SECRET=your-secret-key-here \
AUTH_ENABLED=true \
API_KEYS=test-api-key-123 \
npm start
```

### Development Mode
```bash
npm run dev
```

## 📡 API Endpoints

### Base URL
```
http://localhost:3001
```

### Authentication
All API endpoints (except health checks) require authentication using either:
- **API Key**: `Authorization: ApiKey <your-api-key>`
- **JWT Token**: `Authorization: Bearer <jwt-token>`

### Available Endpoints

#### Health Checks
- `GET /health` - Basic health check
- `GET /api/health` - API health check

#### Authentication
- `POST /api/auth/token` - Get JWT token
  ```json
  {
    "userId": "test-user",
    "scopes": ["robot:read", "robot:control"]
  }
  ```
- `GET /api/auth/verify` - Verify JWT token

#### Robot Control
- `POST /api/vel` - Send velocity command
  ```json
  {
    "vx": 0.1,
    "vy": 0.1,
    "levels": {
      "up": 1,
      "down": 0,
      "left": 0,
      "right": 1
    }
  }
  ```

#### Monitoring
- `GET /api/websocket/stats` - WebSocket connection statistics
- `GET /api/fleet/stats` - Fleet service statistics
- `GET /api/fleet/health` - Fleet service health
- `GET /api/logs` - Velocity command logs

## 🧪 Testing with Postman

1. Import the `postman-collection.json` file into Postman
2. Set the `token` variable with a JWT token from the auth endpoint
3. Run the requests in order:
   - Get Auth Token
   - Verify Token
   - Send Velocity Command
   - Get Logs

## 🔧 Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `NODE_PORT` | Backend API port | 3001 |
| `MONGO_URI` | MongoDB connection string | mongodb://localhost:27017/teleop_db |
| `FLEET_API_URL` | Fleet service URL | http://localhost:8000 |
| `CORS_ORIGIN` | CORS allowed origin | http://localhost:3000 |
| `VX_MAX` | Maximum X velocity | 1.0 |
| `VY_MAX` | Maximum Y velocity | 1.0 |
| `JWT_SECRET` | JWT signing secret | (required) |
| `AUTH_ENABLED` | Enable authentication | true |
| `API_KEYS` | Comma-separated API keys | (required) |

## 📊 Velocity Calculation

The API validates velocity commands by calculating expected values from speed levels:

```
netX = up - down
netY = right - left
vx = clamp(netX, -10, 10) × 0.1
vy = clamp(netY, -10, 10) × 0.1
```

Where:
- `up`, `down`, `left`, `right` are speed levels (0-10)
- `vx`, `vy` are velocity values (-1.0 to 1.0)

## 🐛 Troubleshooting

### Common Issues

1. **"Authentication required" error**
   - Ensure API key is sent with `Authorization: ApiKey <key>` header
   - Check that `API_KEYS` environment variable is set

2. **"JWT_SECRET not configured" error**
   - Set `JWT_SECRET` environment variable

3. **"Validation failed" error for velocity commands**
   - Ensure `vx` and `vy` values match calculated values from levels
   - Use the velocity calculation formula above

4. **MongoDB connection issues**
   - Verify MongoDB is running on port 27017
   - Check `MONGO_URI` environment variable

### Logs
Backend logs are available in the console when running in development mode.

## 🔗 Related Services

- **Fleet Service**: Handles robot communication (port 8000)
- **WebSocket Server**: Real-time communication (same port as API)
- **MongoDB**: Data persistence (port 27017)

## 📝 API Response Format

All API responses follow this format:
```json
{
  "ok": true|false,
  "data": {...},
  "error": "error message",
  "timestamp": "2025-10-18T15:14:19.461Z"
}
```
