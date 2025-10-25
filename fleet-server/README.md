# FastAPI Fleet Server

A scalable FastAPI service for robot fleet management and command distribution via Zenoh messaging.

## 🏗️ Architecture

The Fleet Server follows a clean, modular architecture:

```
fleet-server/
├── app/                    # Main application package
│   ├── api/               # API layer (routes, endpoints)
│   ├── core/              # Core business logic
│   ├── models/            # Pydantic models
│   ├── services/          # External service integrations
│   ├── config/            # Configuration management
│   └── utils/             # Utility functions
├── tests/                 # Comprehensive test suite
├── run.py                 # Application entry point
└── requirements.txt       # Dependencies
```

## ✨ Features

- **🔒 Authentication**: API key and JWT token support with scope-based authorization
- **📡 Zenoh Integration**: Real-time robot communication with automatic reconnection
- **✅ Validation**: Comprehensive velocity range and safety validation
- **📊 Monitoring**: Health checks, status reporting, and connection monitoring
- **🛡️ Error Handling**: Robust error handling with structured responses
- **📝 Logging**: Structured logging with configurable levels
- **🧪 Testing**: Comprehensive test suite with mocking
- **⚙️ Configuration**: Environment-based configuration with validation

## 🚀 API Endpoints

### Health & Status
- `GET /health` - Health check endpoint
- `GET /status` - Detailed status with configuration
- `GET /` - Root endpoint with service information

### Fleet Control (Authenticated)
- `POST /api/v1/fleet/vel` - Send velocity command to robot
- `POST /api/v1/fleet/stop` - Stop robot (zero velocity)

## 🔐 Authentication

The service supports multiple authentication methods:

### API Key Authentication
```bash
curl -H "Authorization: ApiKey your-api-key" \
     -H "Content-Type: application/json" \
     -d '{"vx": 1.0, "vy": 0.5}' \
     http://localhost:8000/api/v1/fleet/vel
```

### JWT Token Authentication
```bash
curl -H "Authorization: Bearer your-jwt-token" \
     -H "Content-Type: application/json" \
     -d '{"vx": 1.0, "vy": 0.5}' \
     http://localhost:8000/api/v1/fleet/vel
```

## ⚙️ Configuration

Configure via environment variables:

```bash
# Server Configuration
FLEET_HOST=0.0.0.0
FLEET_PORT=8000
DEBUG=false

# Velocity Limits
VX_MAX=1.0
VY_MAX=1.0
MAX_LEVEL=10
MIN_LEVEL=0

# Zenoh Configuration
ZENOH_LOCATOR=tcp/localhost:7447
Z_KEY_CMD_VEL=rt/ros2/cmd_vel
ZENOH_CONNECT_TIMEOUT=10.0
ZENOH_RECONNECT_INTERVAL=5.0
ZENOH_MAX_RECONNECT_ATTEMPTS=5

# Authentication
AUTH_ENABLED=true
API_KEYS=key1,key2,key3
JWT_SECRET=your-secret-key
JWT_EXPIRES_IN=24h
JWT_ALGORITHM=HS256

# CORS
CORS_ORIGINS=http://localhost:3000,http://localhost:3001
CORS_ALLOW_CREDENTIALS=true

# Logging
LOG_LEVEL=INFO
LOG_FORMAT="%(asctime)s - %(name)s - %(levelname)s - %(message)s"

# Monitoring
HEALTH_CHECK_INTERVAL=30.0
METRICS_ENABLED=true
```

## 🛠️ Installation & Setup

1. **Install dependencies:**
```bash
pip install -r requirements.txt
```

2. **Set environment variables:**
```bash
cp .env.example .env
# Edit .env with your configuration
```

3. **Run the server:**
```bash
python run.py
```

Or with uvicorn directly:
```bash
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```

## 🧪 Testing

Run the comprehensive test suite:

```bash
# Using the test runner
python run_tests.py

# Or with pytest directly
pytest tests/ -v --asyncio-mode=auto

# Run specific test files
pytest tests/test_api.py -v
pytest tests/test_auth.py -v
```

## 🐳 Docker

Build and run with Docker:

```bash
# Build image
docker build -t fleet-server .

# Run container
docker run -p 8000:8000 --env-file .env fleet-server
```

## 📚 API Documentation

Interactive API documentation is available at:
- **Swagger UI**: `http://localhost:8000/docs`
- **ReDoc**: `http://localhost:8000/redoc`

## 📋 Request/Response Examples

### Velocity Command
```json
{
  "vx": 1.0,
  "vy": 0.5,
  "levels": {
    "up": 5,
    "down": 0,
    "left": 0,
    "right": 3
  },
  "source": "web"
}
```

### Response
```json
{
  "published": true,
  "message": "Velocity command published successfully",
  "timestamp": "2024-01-01T12:00:00.000Z",
  "command": {
    "vx": 1.0,
    "vy": 0.5,
    "levels": {
      "up": 5,
      "down": 0,
      "left": 0,
      "right": 3
    },
    "source": "web"
  }
}
```

### Error Response
```json
{
  "error": "Validation Error",
  "detail": "Data validation failed",
  "errors": [
    "vx: Velocity X must be between -1.0 and 1.0"
  ],
  "timestamp": "2024-01-01T12:00:00.000Z"
}
```

## 🗺️ Coordinate System

- **Forward**: +X direction
- **Right**: +Y direction  
- **Angular Z**: Always 0 (no rotation)

## 📊 Monitoring & Health Checks

The service provides comprehensive monitoring:

- **Health Endpoint**: `/health` - Basic service health
- **Status Endpoint**: `/status` - Detailed service status
- **Zenoh Monitoring**: Automatic connection health checks
- **Structured Logging**: Request/response logging with authentication context
- **Error Tracking**: Comprehensive error logging and reporting

## 🏗️ Development

### Project Structure

- **`app/api/`**: API routes and endpoints
- **`app/core/`**: Core business logic (auth, validation, exceptions)
- **`app/models/`**: Pydantic models for requests/responses
- **`app/services/`**: External service integrations (Zenoh)
- **`app/config/`**: Configuration management
- **`app/utils/`**: Utility functions and helpers
- **`tests/`**: Comprehensive test suite

### Adding New Features

1. **Models**: Add request/response models in `app/models/`
2. **Services**: Add business logic in `app/services/`
3. **Endpoints**: Add API endpoints in `app/api/v1/endpoints/`
4. **Tests**: Add tests in `tests/`
5. **Configuration**: Add config options in `app/config/settings.py`

### Code Quality

The project follows Python best practices:
- **Type Hints**: Full type annotation coverage
- **Pydantic Models**: Request/response validation
- **Dependency Injection**: Clean separation of concerns
- **Error Handling**: Comprehensive exception handling
- **Testing**: High test coverage with mocking
- **Logging**: Structured logging throughout

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License.