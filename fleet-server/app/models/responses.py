"""
Response models for Fleet Server API

Pydantic models for API responses with proper typing and documentation.
"""

from typing import Optional, Dict, Any, List
from pydantic import BaseModel, Field

from .requests import VelocityCommand


class BaseResponse(BaseModel):
    """Base response model with common fields"""
    timestamp: str = Field(..., description="Response timestamp (ISO format)")
    
    model_config = {
        "json_schema_extra": {
            "example": {
                "timestamp": "2024-01-01T12:00:00.000Z"
            }
        }
    }


class VelocityResponse(BaseResponse):
    """Velocity command response model"""
    published: bool = Field(..., description="Whether the command was successfully published")
    message: str = Field(..., description="Response message")
    command: Optional[VelocityCommand] = Field(None, description="The processed command")
    
    model_config = {
        "json_schema_extra": {
            "example": {
                "published": True,
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
        }
    }


class HealthResponse(BaseResponse):
    """Health check response model"""
    status: str = Field(..., description="Service status")
    version: str = Field(..., description="Service version")
    zenoh_connected: bool = Field(..., description="Zenoh connection status")
    
    model_config = {
        "json_schema_extra": {
            "example": {
                "status": "healthy",
                "version": "1.0.0",
                "timestamp": "2024-01-01T12:00:00.000Z",
                "zenoh_connected": True
            }
        }
    }


class StatusResponse(BaseResponse):
    """Detailed status response model"""
    service: str = Field(..., description="Service name")
    status: str = Field(..., description="Service status")
    configuration: Dict[str, Any] = Field(..., description="Service configuration")
    connections: Dict[str, Any] = Field(..., description="Connection status")
    authentication: Dict[str, Any] = Field(..., description="Authentication configuration")
    
    model_config = {
        "json_schema_extra": {
            "example": {
                "service": "fleet-server",
                "status": "running",
                "timestamp": "2024-01-01T12:00:00.000Z",
                "configuration": {
                    "vx_max": 1.0,
                    "vy_max": 1.0,
                    "zenoh_locator": "tcp/localhost:7447",
                    "zenoh_key": "cmd_vel",
                    "auth_enabled": True,
                    "cors_origins": ["http://localhost:3000"]
                },
                "connections": {
                    "zenoh": {
                        "connected": True,
                        "publish_count": 42
                    }
                },
                "authentication": {
                    "enabled": True,
                    "api_keys_configured": True,
                    "jwt_configured": True
                }
            }
        }
    }


class ErrorResponse(BaseResponse):
    """Error response model"""
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Detailed error information")
    errors: Optional[List[str]] = Field(None, description="List of validation errors")
    
    model_config = {
        "json_schema_extra": {
            "example": {
                "error": "Validation Error",
                "detail": "vx must be between -1.0 and 1.0",
                "timestamp": "2024-01-01T12:00:00.000Z",
                "errors": [
                    "vx: Velocity X must be between -1.0 and 1.0"
                ]
            }
        }
    }


class ServiceInfoResponse(BaseResponse):
    """Service information response model"""
    service: str = Field(..., description="Service name")
    version: str = Field(..., description="Service version")
    description: str = Field(..., description="Service description")
    docs: str = Field(..., description="API documentation URL")
    health: str = Field(..., description="Health check endpoint")
    status: str = Field(..., description="Status endpoint")
    endpoints: Dict[str, str] = Field(..., description="Available endpoints")
    
    model_config = {
        "json_schema_extra": {
            "example": {
                "service": "FastAPI Fleet Server",
                "version": "1.0.0",
                "description": "Robot fleet management and command distribution service",
                "docs": "/docs",
                "health": "/health",
                "status": "/status",
                "timestamp": "2024-01-01T12:00:00.000Z",
                "endpoints": {
                    "velocity": "/fleet/vel",
                    "stop": "/fleet/stop"
                }
            }
        }
    }