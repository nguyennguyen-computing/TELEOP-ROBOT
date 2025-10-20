"""
API v1 router for Fleet Server

Combines all v1 endpoints into a single router.
"""

from fastapi import APIRouter

from app.api.v1.endpoints import health, fleet

api_router = APIRouter()

# Include health endpoints at root level
api_router.include_router(health.router, tags=["Health"])

# Include fleet endpoints under /fleet prefix
api_router.include_router(fleet.router, prefix="/fleet", tags=["Fleet Control"])