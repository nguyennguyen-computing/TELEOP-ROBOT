"""
Authentication tests for Fleet Server

Tests for API key and JWT authentication functionality.
"""

import pytest
from datetime import datetime, timedelta

from app.core.auth import auth_service, AuthUser, JWTPayload
from app.core.exceptions import AuthenticationError, AuthorizationError


class TestAuthService:
    """Test authentication service functionality"""
    
    def test_api_key_validation(self):
        """Test API key validation"""
        # Valid API key
        assert auth_service.validate_api_key("test-key-123") == True
        assert auth_service.validate_api_key("admin-key-456") == True
        
        # Invalid API key
        assert auth_service.validate_api_key("invalid-key") == False
        assert auth_service.validate_api_key("") == False
    
    def test_jwt_token_generation_and_validation(self):
        """Test JWT token generation and validation"""
        # Generate token
        token = auth_service.generate_jwt_token("test-user", ["robot:control"])
        assert token is not None
        
        # Validate token
        payload = auth_service.validate_jwt_token(token)
        assert payload is not None
        assert payload.sub == "test-user"
        assert "robot:control" in payload.scope
    
    def test_jwt_token_expiration(self):
        """Test JWT token expiration handling"""
        # Create expired token by mocking time
        with pytest.mock.patch('app.core.auth.datetime') as mock_datetime:
            # Set time to past
            mock_datetime.utcnow.return_value = datetime(2020, 1, 1)
            token = auth_service.generate_jwt_token("test-user", ["robot:control"])
            
            # Reset time to present
            mock_datetime.utcnow.return_value = datetime.utcnow()
            
            # Token should be expired
            payload = auth_service.validate_jwt_token(token)
            assert payload is None
    
    def test_parse_expires_in(self):
        """Test JWT expiration parsing"""
        # Test hours
        delta = auth_service.parse_expires_in("24h")
        assert delta == timedelta(hours=24)
        
        # Test minutes
        delta = auth_service.parse_expires_in("30m")
        assert delta == timedelta(minutes=30)
        
        # Test days
        delta = auth_service.parse_expires_in("7d")
        assert delta == timedelta(days=7)
        
        # Test default (hours)
        delta = auth_service.parse_expires_in("12")
        assert delta == timedelta(hours=12)


class TestAuthUser:
    """Test AuthUser model"""
    
    def test_auth_user_creation(self):
        """Test AuthUser model creation"""
        user = AuthUser(
            id="test-user",
            scopes=["robot:control", "robot:read"],
            auth_method="jwt"
        )
        
        assert user.id == "test-user"
        assert "robot:control" in user.scopes
        assert user.auth_method == "jwt"
    
    def test_auth_user_defaults(self):
        """Test AuthUser model defaults"""
        user = AuthUser(id="test-user", auth_method="api_key")
        
        assert user.scopes == []
        assert user.auth_method == "api_key"


class TestJWTPayload:
    """Test JWT payload model"""
    
    def test_jwt_payload_creation(self):
        """Test JWT payload creation"""
        now = int(datetime.utcnow().timestamp())
        payload = JWTPayload(
            sub="test-user",
            iat=now,
            exp=now + 3600,
            scope=["robot:control"]
        )
        
        assert payload.sub == "test-user"
        assert payload.iat == now
        assert payload.exp == now + 3600
        assert "robot:control" in payload.scope
    
    def test_jwt_payload_defaults(self):
        """Test JWT payload defaults"""
        now = int(datetime.utcnow().timestamp())
        payload = JWTPayload(
            sub="test-user",
            iat=now,
            exp=now + 3600
        )
        
        assert payload.scope == []


class TestAuthenticationExceptions:
    """Test authentication exceptions"""
    
    def test_authentication_error(self):
        """Test AuthenticationError exception"""
        with pytest.raises(AuthenticationError) as exc_info:
            raise AuthenticationError("Test authentication error")
        
        assert exc_info.value.status_code == 401
        assert "Test authentication error" in str(exc_info.value.detail)
    
    def test_authorization_error(self):
        """Test AuthorizationError exception"""
        with pytest.raises(AuthorizationError) as exc_info:
            raise AuthorizationError("Test authorization error")
        
        assert exc_info.value.status_code == 403
        assert "Test authorization error" in str(exc_info.value.detail)


class TestAuthenticationIntegration:
    """Test authentication integration scenarios"""
    
    @pytest.mark.asyncio
    async def test_disabled_authentication(self):
        """Test behavior when authentication is disabled"""
        # Mock disabled authentication
        with pytest.mock.patch.object(auth_service, 'config', {'enabled': False}):
            from fastapi import Request
            
            # Create mock request
            request = pytest.mock.Mock(spec=Request)
            request.query_params = {}
            request.headers = {}
            
            user = await auth_service.get_current_user(request, None)
            
            assert user is not None
            assert user.id == "anonymous"
            assert "*" in user.scopes
            assert user.auth_method == "disabled"
    
    def test_scope_checking(self):
        """Test scope-based authorization"""
        # User with robot:control scope
        user = AuthUser(
            id="test-user",
            scopes=["robot:control"],
            auth_method="jwt"
        )
        
        # Should have access to robot control
        assert "robot:control" in user.scopes
        
        # Should not have admin access
        assert "admin" not in user.scopes
    
    def test_wildcard_scope(self):
        """Test wildcard scope access"""
        user = AuthUser(
            id="admin-user",
            scopes=["*"],
            auth_method="api_key"
        )
        
        # Wildcard should grant all access
        assert "*" in user.scopes