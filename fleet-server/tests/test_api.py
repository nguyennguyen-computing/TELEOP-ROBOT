"""
API endpoint tests for Fleet Server

Tests for all API endpoints including authentication and validation.
"""

import pytest
from fastapi.testclient import TestClient
from httpx import AsyncClient


class TestHealthEndpoints:
    """Test health and status endpoints"""
    
    def test_health_endpoint(self, client: TestClient, mock_zenoh_service):
        """Test health check endpoint"""
        response = client.get("/health")
        assert response.status_code == 200
        
        data = response.json()
        assert data["status"] == "healthy"
        assert "timestamp" in data
        assert "zenoh_connected" in data
    
    def test_status_endpoint(self, client: TestClient, mock_zenoh_service):
        """Test status endpoint"""
        response = client.get("/status")
        assert response.status_code == 200
        
        data = response.json()
        assert data["service"] == "fleet-server"
        assert data["status"] == "running"
        assert "configuration" in data
        assert "connections" in data
        assert "authentication" in data
    
    def test_root_endpoint(self, client: TestClient):
        """Test root endpoint"""
        response = client.get("/")
        assert response.status_code == 200
        
        data = response.json()
        assert "service" in data
        assert "version" in data
        assert "endpoints" in data


class TestFleetEndpoints:
    """Test fleet control endpoints"""
    
    def test_velocity_endpoint_without_auth(self, client: TestClient, valid_velocity_command):
        """Test velocity endpoint without authentication"""
        response = client.post("/api/v1/fleet/vel", json=valid_velocity_command)
        assert response.status_code == 401
    
    def test_velocity_endpoint_with_api_key(
        self, 
        client: TestClient, 
        valid_velocity_command, 
        api_key_headers,
        mock_zenoh_service
    ):
        """Test velocity endpoint with API key authentication"""
        response = client.post(
            "/api/v1/fleet/vel", 
            json=valid_velocity_command, 
            headers=api_key_headers
        )
        assert response.status_code == 200
        
        data = response.json()
        assert data["published"] == True
        assert "command" in data
        assert "timestamp" in data
    
    def test_velocity_endpoint_with_jwt(
        self, 
        client: TestClient, 
        valid_velocity_command, 
        jwt_headers,
        mock_zenoh_service
    ):
        """Test velocity endpoint with JWT authentication"""
        response = client.post(
            "/api/v1/fleet/vel", 
            json=valid_velocity_command, 
            headers=jwt_headers
        )
        assert response.status_code == 200
        
        data = response.json()
        assert data["published"] == True
    
    def test_velocity_endpoint_validation_error(
        self, 
        client: TestClient, 
        invalid_velocity_command, 
        api_key_headers
    ):
        """Test velocity endpoint with validation errors"""
        response = client.post(
            "/api/v1/fleet/vel", 
            json=invalid_velocity_command, 
            headers=api_key_headers
        )
        assert response.status_code == 400
        
        data = response.json()
        assert "error" in data
    
    def test_stop_endpoint(
        self, 
        client: TestClient, 
        api_key_headers,
        mock_zenoh_service
    ):
        """Test robot stop endpoint"""
        response = client.post("/api/v1/fleet/stop", headers=api_key_headers)
        assert response.status_code == 200
        
        data = response.json()
        assert data["published"] == True
        assert data["command"]["vx"] == 0.0
        assert data["command"]["vy"] == 0.0
    
    def test_invalid_authentication(
        self, 
        client: TestClient, 
        valid_velocity_command, 
        invalid_headers
    ):
        """Test endpoints with invalid authentication"""
        response = client.post(
            "/api/v1/fleet/vel", 
            json=valid_velocity_command, 
            headers=invalid_headers
        )
        assert response.status_code == 401


class TestErrorHandling:
    """Test error handling scenarios"""
    
    def test_invalid_json_request(self, client: TestClient, api_key_headers):
        """Test handling of invalid JSON requests"""
        response = client.post(
            "/api/v1/fleet/vel", 
            data="invalid json", 
            headers=api_key_headers
        )
        assert response.status_code == 422
    
    def test_missing_required_fields(self, client: TestClient, api_key_headers):
        """Test handling of missing required fields"""
        command = {"vx": 1.0}  # Missing vy
        
        response = client.post(
            "/api/v1/fleet/vel", 
            json=command, 
            headers=api_key_headers
        )
        assert response.status_code == 422
    
    def test_zenoh_service_unavailable(
        self, 
        client: TestClient, 
        valid_velocity_command, 
        api_key_headers
    ):
        """Test handling when Zenoh service is unavailable"""
        with pytest.mock.patch('app.services.zenoh_service.publish_velocity', return_value=False):
            response = client.post(
                "/api/v1/fleet/vel", 
                json=valid_velocity_command, 
                headers=api_key_headers
            )
            assert response.status_code == 503


@pytest.mark.asyncio
class TestAsyncEndpoints:
    """Test async endpoint functionality"""
    
    async def test_async_health_check(self, async_client: AsyncClient, mock_zenoh_service):
        """Test health check with async client"""
        response = await async_client.get("/health")
        assert response.status_code == 200
        
        data = response.json()
        assert data["status"] == "healthy"
    
    async def test_async_velocity_command(
        self, 
        async_client: AsyncClient, 
        valid_velocity_command, 
        api_key_headers,
        mock_zenoh_service
    ):
        """Test velocity command with async client"""
        response = await async_client.post(
            "/api/v1/fleet/vel", 
            json=valid_velocity_command, 
            headers=api_key_headers
        )
        assert response.status_code == 200
        
        data = response.json()
        assert data["published"] == True