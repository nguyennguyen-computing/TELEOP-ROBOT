"""
Test configuration for Fleet Server

Pytest fixtures and test setup.
"""

import pytest
import asyncio
import os
from typing import Generator, AsyncGenerator
from unittest.mock import Mock, AsyncMock, patch

from fastapi.testclient import TestClient
from httpx import AsyncClient

# Set test environment variables
os.environ.update({
    "AUTH_ENABLED": "true",
    "API_KEYS": "test-key-123,admin-key-456",
    "JWT_SECRET": "test-secret-key-for-testing-only",
    "VX_MAX": "2.0",
    "VY_MAX": "1.5",
    "ZENOH_LOCATOR": "tcp/localhost:7447",
    "Z_KEY_CMD_VEL": "test/cmd_vel",
    "DEBUG": "true",
    "LOG_LEVEL": "DEBUG",
})

from app.main import app
from app.core.auth import auth_service
from app.services import zenoh_service


@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture
def client() -> Generator[TestClient, None, None]:
    """Create test client"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
async def async_client() -> AsyncGenerator[AsyncClient, None]:
    """Create async test client"""
    async with AsyncClient(app=app, base_url="http://test") as ac:
        yield ac


@pytest.fixture
def mock_zenoh_service():
    """Mock Zenoh service for testing"""
    with patch.object(zenoh_service, 'publish_velocity', new_callable=AsyncMock) as mock_publish:
        with patch.object(zenoh_service, 'health_check', new_callable=AsyncMock) as mock_health:
            with patch.object(zenoh_service, 'get_status') as mock_status:
                mock_publish.return_value = True
                mock_health.return_value = True
                mock_status.return_value = {
                    "connected": True,
                    "publish_count": 0,
                    "last_publish_time": None
                }
                yield {
                    "publish_velocity": mock_publish,
                    "health_check": mock_health,
                    "get_status": mock_status
                }


@pytest.fixture
def valid_api_key() -> str:
    """Valid API key for testing"""
    return "test-key-123"


@pytest.fixture
def invalid_api_key() -> str:
    """Invalid API key for testing"""
    return "invalid-key"


@pytest.fixture
def valid_jwt_token() -> str:
    """Valid JWT token for testing"""
    return auth_service.generate_jwt_token("test-user", ["robot:control"])


@pytest.fixture
def invalid_jwt_token() -> str:
    """Invalid JWT token for testing"""
    return "invalid.jwt.token"


@pytest.fixture
def valid_velocity_command() -> dict:
    """Valid velocity command for testing"""
    return {
        "vx": 1.0,
        "vy": 0.5,
        "levels": {
            "up": 5,
            "down": 0,
            "left": 0,
            "right": 3
        },
        "source": "test"
    }


@pytest.fixture
def invalid_velocity_command() -> dict:
    """Invalid velocity command for testing"""
    return {
        "vx": 10.0,  # Exceeds VX_MAX
        "vy": 0.5,
        "levels": {
            "up": 5,
            "down": 0,
            "left": 0,
            "right": 3
        }
    }


@pytest.fixture
def api_key_headers(valid_api_key: str) -> dict:
    """Headers with valid API key"""
    return {"Authorization": f"ApiKey {valid_api_key}"}


@pytest.fixture
def jwt_headers(valid_jwt_token: str) -> dict:
    """Headers with valid JWT token"""
    return {"Authorization": f"Bearer {valid_jwt_token}"}


@pytest.fixture
def invalid_headers(invalid_api_key: str) -> dict:
    """Headers with invalid API key"""
    return {"Authorization": f"ApiKey {invalid_api_key}"}