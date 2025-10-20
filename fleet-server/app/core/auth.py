"""
Authentication module for Fleet Server

Provides API key and JWT token validation for secure access to fleet endpoints.
Supports both API key-based authentication for service-to-service communication
and JWT token-based authentication for user sessions.
"""

import jwt
import logging
from datetime import datetime, timedelta
from typing import Optional, List, Dict, Any
from functools import wraps

from fastapi import Depends, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel

from app.config import settings
from app.core.exceptions import AuthenticationError, AuthorizationError

logger = logging.getLogger(__name__)

# Security scheme
security = HTTPBearer(auto_error=False)


class JWTPayload(BaseModel):
    """JWT token payload model"""
    sub: str  # Subject (user ID)
    iat: int  # Issued at
    exp: int  # Expiration time
    scope: List[str] = []  # Permissions/scopes


class AuthUser(BaseModel):
    """Authenticated user model"""
    id: str
    scopes: List[str] = []
    auth_method: str


class AuthRequest(BaseModel):
    """Authentication request context"""
    api_key: Optional[str] = None
    token: Optional[str] = None
    user: Optional[AuthUser] = None


class AuthService:
    """Authentication service"""
    
    def __init__(self):
        self.config = settings.get_auth_config()
        logger.info(f"Authentication service initialized (enabled: {self.config['enabled']})")
    
    def parse_expires_in(self, expires_in: str) -> timedelta:
        """Parse JWT expiration string to timedelta"""
        if expires_in.endswith('h'):
            hours = int(expires_in[:-1])
            return timedelta(hours=hours)
        elif expires_in.endswith('m'):
            minutes = int(expires_in[:-1])
            return timedelta(minutes=minutes)
        elif expires_in.endswith('d'):
            days = int(expires_in[:-1])
            return timedelta(days=days)
        else:
            # Default to hours if no unit specified
            return timedelta(hours=int(expires_in))
    
    def generate_jwt_token(self, user_id: str, scopes: List[str] = None) -> str:
        """Generate JWT token for user"""
        if scopes is None:
            scopes = []
        
        now = datetime.utcnow()
        expires_delta = self.parse_expires_in(self.config['jwt_expires_in'])
        
        payload = {
            "sub": user_id,
            "iat": int(now.timestamp()),
            "exp": int((now + expires_delta).timestamp()),
            "scope": scopes
        }
        
        token = jwt.encode(payload, self.config['jwt_secret'], algorithm=self.config['jwt_algorithm'])
        logger.info(f"Generated JWT token for user {user_id} with scopes {scopes}")
        return token
    
    def validate_jwt_token(self, token: str) -> Optional[JWTPayload]:
        """Validate JWT token and return payload"""
        try:
            payload = jwt.decode(
                token, 
                self.config['jwt_secret'], 
                algorithms=[self.config['jwt_algorithm']]
            )
            
            # Check expiration
            if payload.get("exp", 0) < datetime.utcnow().timestamp():
                logger.warning("JWT token expired")
                return None
            
            return JWTPayload(**payload)
        
        except jwt.InvalidTokenError as e:
            logger.warning(f"Invalid JWT token: {e}")
            return None
        except Exception as e:
            logger.error(f"JWT validation error: {e}")
            return None
    
    def validate_api_key(self, api_key: str) -> bool:
        """Validate API key"""
        if not self.config['api_keys']:
            logger.warning("No API keys configured")
            return False
        
        is_valid = api_key in self.config['api_keys']
        if is_valid:
            logger.info(f"Valid API key used: {api_key[:8]}...")
        else:
            logger.warning(f"Invalid API key attempted: {api_key[:8]}...")
        
        return is_valid
    
    def extract_auth_from_request(self, request: Request) -> AuthRequest:
        """Extract authentication information from request"""
        auth_req = AuthRequest()
        
        # Check for API key in query parameters
        api_key = request.query_params.get("api_key")
        if api_key:
            auth_req.api_key = api_key
        
        # Check for API key in headers
        auth_header = request.headers.get("authorization", "")
        if auth_header.startswith("ApiKey "):
            auth_req.api_key = auth_header[7:]  # Remove "ApiKey " prefix
        elif auth_header.startswith("Bearer "):
            auth_req.token = auth_header[7:]  # Remove "Bearer " prefix
        
        return auth_req
    
    async def get_current_user(
        self,
        request: Request,
        credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
    ) -> Optional[AuthUser]:
        """Get current authenticated user"""
        
        # If authentication is disabled, return a default user
        if not self.config['enabled']:
            return AuthUser(
                id="anonymous",
                scopes=["*"],
                auth_method="disabled"
            )
        
        # Extract authentication information
        auth_req = self.extract_auth_from_request(request)
        
        # Override with credentials if provided
        if credentials:
            if credentials.scheme.lower() == "bearer":
                auth_req.token = credentials.credentials
            elif credentials.scheme.lower() == "apikey":
                auth_req.api_key = credentials.credentials
        
        # Validate API key
        if auth_req.api_key:
            if self.validate_api_key(auth_req.api_key):
                return AuthUser(
                    id=f"api_key_{auth_req.api_key[:8]}",
                    scopes=["robot:control", "robot:read"],
                    auth_method="api_key"
                )
            else:
                raise AuthenticationError("Invalid API key")
        
        # Validate JWT token
        if auth_req.token:
            payload = self.validate_jwt_token(auth_req.token)
            if payload:
                return AuthUser(
                    id=payload.sub,
                    scopes=payload.scope,
                    auth_method="jwt"
                )
            else:
                raise AuthenticationError("Invalid or expired JWT token")
        
        # No valid authentication found
        raise AuthenticationError("Authentication required")
    
    def require_auth(self, scopes: List[str] = None):
        """Dependency to require authentication with optional scope checking"""
        
        async def auth_dependency(user: AuthUser = Depends(self.get_current_user)) -> AuthUser:
            if not user:
                raise AuthenticationError()
            
            # Check scopes if specified
            if scopes and "*" not in user.scopes:
                if not any(scope in user.scopes for scope in scopes):
                    raise AuthorizationError(
                        f"Required scopes: {scopes}. User scopes: {user.scopes}"
                    )
            
            logger.info(f"Authenticated user: {user.id} ({user.auth_method})")
            return user
        
        return auth_dependency
    
    def require_scope(self, required_scopes: List[str]):
        """Decorator to require specific scopes"""
        
        def decorator(func):
            @wraps(func)
            async def wrapper(*args, **kwargs):
                # Get user from kwargs (should be injected by FastAPI)
                user = kwargs.get('user') or kwargs.get('current_user')
                if not user:
                    raise AuthenticationError("User not found in request context")
                
                # Check scopes
                if "*" not in user.scopes:
                    if not any(scope in user.scopes for scope in required_scopes):
                        raise AuthorizationError(
                            f"Required scopes: {required_scopes}. User scopes: {user.scopes}"
                        )
                
                return await func(*args, **kwargs)
            return wrapper
        return decorator


# Global auth service instance
auth_service = AuthService()

# Common scope requirements
ROBOT_CONTROL_SCOPES = ["robot:control"]
ROBOT_READ_SCOPES = ["robot:read"]
ADMIN_SCOPES = ["admin"]

# Pre-configured dependencies
RequireAuth = Depends(auth_service.require_auth())
RequireRobotControl = Depends(auth_service.require_auth(ROBOT_CONTROL_SCOPES))
RequireRobotRead = Depends(auth_service.require_auth(ROBOT_READ_SCOPES))
RequireAdmin = Depends(auth_service.require_auth(ADMIN_SCOPES))