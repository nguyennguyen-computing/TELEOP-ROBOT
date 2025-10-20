"""
API dependencies for Fleet Server

Common dependencies used across API endpoints.
"""

from typing import Generator
from fastapi import Depends

from app.core.auth import auth_service, AuthUser
from app.core.validation import validation_service
from app.services import zenoh_service


def get_auth_service():
    """Get authentication service dependency"""
    return auth_service


def get_validation_service():
    """Get validation service dependency"""
    return validation_service


def get_zenoh_service():
    """Get Zenoh service dependency"""
    return zenoh_service


# Import the pre-configured dependencies from auth module
from app.core.auth import RequireAuth, RequireRobotControl, RequireRobotRead, RequireAdmin

# Re-export for convenience
def get_current_user():
    """Get current authenticated user dependency"""
    return RequireAuth

def require_robot_control():
    """Require robot control permissions dependency"""
    return RequireRobotControl

def require_robot_read():
    """Require robot read permissions dependency"""
    return RequireRobotRead

def require_admin():
    """Require admin permissions dependency"""
    return RequireAdmin