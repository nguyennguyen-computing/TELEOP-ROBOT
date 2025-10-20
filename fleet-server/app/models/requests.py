"""
Request models for Fleet Server API

Pydantic models for validating incoming API requests.
"""

from typing import Optional
from pydantic import BaseModel, Field, validator

from app.config import settings


class SpeedLevels(BaseModel):
    """Speed levels for directional control"""
    up: int = Field(..., ge=0, le=10, description="Forward level (0-10)")
    down: int = Field(..., ge=0, le=10, description="Backward level (0-10)")
    left: int = Field(..., ge=0, le=10, description="Left level (0-10)")
    right: int = Field(..., ge=0, le=10, description="Right level (0-10)")
    
    @validator('up', 'down', 'left', 'right')
    def validate_level_range(cls, v):
        """Validate level is within configured range"""
        if v < settings.MIN_LEVEL or v > settings.MAX_LEVEL:
            raise ValueError(f'Level must be between {settings.MIN_LEVEL} and {settings.MAX_LEVEL}')
        return v


class VelocityCommand(BaseModel):
    """Velocity command request model"""
    vx: float = Field(..., description="Velocity X component (m/s)")
    vy: float = Field(..., description="Velocity Y component (m/s)")
    levels: Optional[SpeedLevels] = Field(None, description="Speed levels for each direction")
    source: Optional[str] = Field("fleet", description="Command source identifier")
    
    @validator('vx')
    def validate_vx(cls, v):
        """Validate vx is within configured limits"""
        if abs(v) > settings.VX_MAX:
            raise ValueError(f'vx must be between -{settings.VX_MAX} and {settings.VX_MAX}')
        return v
    
    @validator('vy')
    def validate_vy(cls, v):
        """Validate vy is within configured limits"""
        if abs(v) > settings.VY_MAX:
            raise ValueError(f'vy must be between -{settings.VY_MAX} and {settings.VY_MAX}')
        return v
    
    model_config = {
        "json_schema_extra": {
            "example": {
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


class StopCommand(BaseModel):
    """Stop command request model"""
    source: Optional[str] = Field("fleet", description="Command source identifier")
    
    model_config = {
        "json_schema_extra": {
            "example": {
                "source": "emergency_stop"
            }
        }
    }