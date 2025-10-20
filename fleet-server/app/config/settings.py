"""
Configuration settings for Fleet Server

Centralized configuration management using Pydantic settings.
Supports environment variables and validation.
"""

import os
from typing import List, Optional
from pydantic import Field, validator
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application settings with environment variable support"""
    
    # Application metadata
    APP_NAME: str = "Fleet Server API"
    APP_VERSION: str = "1.0.0"
    APP_DESCRIPTION: str = "FastAPI service for robot fleet management and command distribution"
    
    # Server configuration
    HOST: str = Field(default="0.0.0.0", env="FLEET_HOST")
    PORT: int = Field(default=8000, env="FLEET_PORT")
    DEBUG: bool = Field(default=False, env="DEBUG")
    RELOAD: bool = Field(default=False, env="RELOAD")
    
    # Velocity limits
    VX_MAX: float = Field(default=1.0, env="VX_MAX", description="Maximum X velocity (m/s)")
    VY_MAX: float = Field(default=1.0, env="VY_MAX", description="Maximum Y velocity (m/s)")
    
    # Speed level configuration
    MAX_LEVEL: int = Field(default=10, description="Maximum speed level")
    MIN_LEVEL: int = Field(default=0, description="Minimum speed level")
    
    # CORS configuration
    CORS_ORIGINS: str = Field(
        default="http://localhost:3000",
        env="CORS_ORIGINS",
        description="Allowed CORS origins (comma-separated)"
    )
    CORS_ALLOW_CREDENTIALS: bool = Field(default=True, env="CORS_ALLOW_CREDENTIALS")
    CORS_ALLOW_METHODS: List[str] = Field(
        default=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
        env="CORS_ALLOW_METHODS"
    )
    CORS_ALLOW_HEADERS: List[str] = Field(default=["*"], env="CORS_ALLOW_HEADERS")
    
    # Zenoh configuration
    ZENOH_LOCATOR: str = Field(default="tcp/zenoh:7447", env="ZENOH_LOCATOR")
    Z_KEY_CMD_VEL: str = Field(default="rt/ros2/cmd_vel", env="Z_KEY_CMD_VEL")
    ZENOH_CONNECT_TIMEOUT: float = Field(default=10.0, env="ZENOH_CONNECT_TIMEOUT")
    ZENOH_RECONNECT_INTERVAL: float = Field(default=5.0, env="ZENOH_RECONNECT_INTERVAL")
    ZENOH_MAX_RECONNECT_ATTEMPTS: int = Field(default=5, env="ZENOH_MAX_RECONNECT_ATTEMPTS")
    
    # Authentication configuration
    AUTH_ENABLED: bool = Field(default=False, env="AUTH_ENABLED")
    API_KEYS: str = Field(default="", env="API_KEYS")
    JWT_SECRET: str = Field(default="dev-secret-change-in-production", env="JWT_SECRET")
    JWT_EXPIRES_IN: str = Field(default="24h", env="JWT_EXPIRES_IN")
    JWT_ALGORITHM: str = Field(default="HS256", env="JWT_ALGORITHM")
    
    # Rate limiting configuration
    RATE_LIMIT_WINDOW_MS: int = Field(default=900000, env="RATE_LIMIT_WINDOW_MS")  # 15 minutes
    RATE_LIMIT_MAX_REQUESTS: int = Field(default=100, env="RATE_LIMIT_MAX_REQUESTS")
    
    # Logging configuration
    LOG_LEVEL: str = Field(default="INFO", env="LOG_LEVEL")
    LOG_FORMAT: str = Field(
        default="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        env="LOG_FORMAT"
    )
    
    # Monitoring configuration
    HEALTH_CHECK_INTERVAL: float = Field(default=30.0, env="HEALTH_CHECK_INTERVAL")
    METRICS_ENABLED: bool = Field(default=True, env="METRICS_ENABLED")
    
    @property
    def cors_origins_list(self) -> List[str]:
        """Get CORS origins as a list"""
        if not self.CORS_ORIGINS.strip():
            return ["http://localhost:3000"]
        return [origin.strip() for origin in self.CORS_ORIGINS.split(",") if origin.strip()]
    
    @property
    def api_keys_list(self) -> List[str]:
        """Get API keys as a list"""
        if not self.API_KEYS.strip():
            return []
        return [key.strip() for key in self.API_KEYS.split(",") if key.strip()]
    
    @validator("CORS_ALLOW_METHODS", pre=True)
    def parse_cors_methods(cls, v):
        """Parse CORS methods from string or list"""
        if isinstance(v, str):
            return [method.strip() for method in v.split(",") if method.strip()]
        return v
    
    @validator("CORS_ALLOW_HEADERS", pre=True)
    def parse_cors_headers(cls, v):
        """Parse CORS headers from string or list"""
        if isinstance(v, str):
            return [header.strip() for header in v.split(",") if header.strip()]
        return v
    
    @validator("VX_MAX", "VY_MAX")
    def validate_velocity_limits(cls, v):
        """Validate velocity limits are positive"""
        if v <= 0:
            raise ValueError("Velocity limits must be positive")
        return v
    
    @validator("JWT_SECRET")
    def validate_jwt_secret(cls, v, values):
        """Validate JWT secret is not default in production"""
        debug = values.get("DEBUG", False)
        auth_enabled = values.get("AUTH_ENABLED", False)
        
        # Only validate JWT secret if authentication is enabled and not in debug mode
        if auth_enabled and v == "dev-secret-change-in-production" and not debug:
            raise ValueError("JWT secret must be changed in production when authentication is enabled")
        return v
    
    @property
    def step_x(self) -> float:
        """Calculate X velocity step size"""
        return self.VX_MAX / self.MAX_LEVEL
    
    @property
    def step_y(self) -> float:
        """Calculate Y velocity step size"""
        return self.VY_MAX / self.MAX_LEVEL
    
    @property
    def is_development(self) -> bool:
        """Check if running in development mode"""
        return self.DEBUG or self.RELOAD
    
    def get_zenoh_config(self) -> dict:
        """Get Zenoh configuration dictionary"""
        return {
            "locator": self.ZENOH_LOCATOR,
            "cmd_vel_key": self.Z_KEY_CMD_VEL,
            "connect_timeout": self.ZENOH_CONNECT_TIMEOUT,
            "reconnect_interval": self.ZENOH_RECONNECT_INTERVAL,
            "max_reconnect_attempts": self.ZENOH_MAX_RECONNECT_ATTEMPTS,
        }
    
    def get_auth_config(self) -> dict:
        """Get authentication configuration dictionary"""
        return {
            "enabled": self.AUTH_ENABLED,
            "api_keys": self.api_keys_list,
            "jwt_secret": self.JWT_SECRET,
            "jwt_expires_in": self.JWT_EXPIRES_IN,
            "jwt_algorithm": self.JWT_ALGORITHM,
        }
    
    def get_cors_config(self) -> dict:
        """Get CORS configuration dictionary"""
        return {
            "allow_origins": self.cors_origins_list,
            "allow_credentials": self.CORS_ALLOW_CREDENTIALS,
            "allow_methods": self.CORS_ALLOW_METHODS,
            "allow_headers": self.CORS_ALLOW_HEADERS,
        }
    
    model_config = {
        "env_file": ".env",
        "env_file_encoding": "utf-8",
        "case_sensitive": True,
        "extra": "ignore"  # Allow extra fields from environment
    }


# Global settings instance
settings = Settings()