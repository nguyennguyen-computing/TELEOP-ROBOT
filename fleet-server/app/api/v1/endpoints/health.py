"""
Health and status endpoints for Fleet Server API

Provides health checks, status information, and service metadata.
"""

import logging
from datetime import datetime
from typing import Dict, Any

from fastapi import APIRouter, Depends

from app.config import settings
from app.models.responses import HealthResponse, StatusResponse, ServiceInfoResponse
from app.api.deps import get_zenoh_service
from app.services import ZenohService

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check(
    zenoh_svc: ZenohService = Depends(get_zenoh_service)
):
    """Health check endpoint to verify service status"""
    
    # Check Zenoh connection status
    zenoh_connected = await zenoh_svc.health_check()
    
    return HealthResponse(
        status="healthy",
        version=settings.APP_VERSION,
        timestamp=datetime.utcnow().isoformat(),
        zenoh_connected=zenoh_connected
    )


@router.get("/status", response_model=StatusResponse, tags=["Health"])
async def status_check(
    zenoh_svc: ZenohService = Depends(get_zenoh_service)
):
    """Detailed status endpoint with configuration information"""
    
    # Get Zenoh service status
    zenoh_status = zenoh_svc.get_status()
    
    return StatusResponse(
        service="fleet-server",
        status="running",
        timestamp=datetime.utcnow().isoformat(),
        configuration={
            "vx_max": settings.VX_MAX,
            "vy_max": settings.VY_MAX,
            "zenoh_locator": settings.ZENOH_LOCATOR,
            "zenoh_key": settings.Z_KEY_CMD_VEL,
            "auth_enabled": settings.AUTH_ENABLED,
            "cors_origins": settings.CORS_ORIGINS,
            "max_level": settings.MAX_LEVEL,
            "min_level": settings.MIN_LEVEL,
            "step_x": settings.step_x,
            "step_y": settings.step_y,
        },
        connections={
            "zenoh": zenoh_status
        },
        authentication={
            "enabled": settings.AUTH_ENABLED,
            "api_keys_configured": len(settings.API_KEYS) > 0,
            "jwt_configured": bool(settings.JWT_SECRET),
            "jwt_algorithm": settings.JWT_ALGORITHM,
        }
    )


@router.get("/", response_model=ServiceInfoResponse, tags=["Root"])
async def service_info():
    """Root endpoint with basic service information"""
    return ServiceInfoResponse(
        service=settings.APP_NAME,
        version=settings.APP_VERSION,
        description=settings.APP_DESCRIPTION,
        docs="/docs",
        health="/health",
        status="/status",
        timestamp=datetime.utcnow().isoformat(),
        endpoints={
            "velocity": "/api/v1/fleet/vel",
            "stop": "/api/v1/fleet/stop",
            "health": "/health",
            "status": "/status"
        }
    )