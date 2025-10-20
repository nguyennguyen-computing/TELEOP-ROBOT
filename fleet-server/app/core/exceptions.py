"""
Custom exceptions for Fleet Server

Defines application-specific exceptions with proper error codes and messages.
"""

from typing import Any, Dict, Optional
from fastapi import HTTPException, status


class FleetServerException(Exception):
    """Base exception for Fleet Server"""
    
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        self.message = message
        self.details = details or {}
        super().__init__(self.message)


class AuthenticationError(HTTPException):
    """Authentication failed exception"""
    
    def __init__(self, detail: str = "Authentication failed", headers: Optional[Dict[str, str]] = None):
        super().__init__(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=detail,
            headers=headers or {"WWW-Authenticate": "Bearer"}
        )


class AuthorizationError(HTTPException):
    """Authorization failed exception"""
    
    def __init__(self, detail: str = "Insufficient permissions"):
        super().__init__(
            status_code=status.HTTP_403_FORBIDDEN,
            detail=detail
        )


class ValidationError(HTTPException):
    """Validation failed exception"""
    
    def __init__(self, detail: str, validation_errors: Optional[list] = None):
        self.validation_errors = validation_errors or []
        super().__init__(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "message": detail,
                "errors": self.validation_errors
            }
        )


class ZenohConnectionError(FleetServerException):
    """Zenoh connection error"""
    
    def __init__(self, message: str = "Zenoh connection failed", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, details)


class RobotCommandError(HTTPException):
    """Robot command execution error"""
    
    def __init__(self, detail: str = "Failed to execute robot command"):
        super().__init__(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=detail
        )


class ConfigurationError(FleetServerException):
    """Configuration error"""
    
    def __init__(self, message: str = "Configuration error", details: Optional[Dict[str, Any]] = None):
        super().__init__(message, details)