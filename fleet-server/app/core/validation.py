"""
Validation module for Fleet Server

Provides velocity command validation, range checking, and error handling
for robot control commands. Ensures all velocity commands are within
safe operational limits before forwarding to the robot.
"""

import logging
from typing import List, Dict, Any, Optional, Tuple
from datetime import datetime

from pydantic import BaseModel

from app.config import settings
from app.core.exceptions import ValidationError

logger = logging.getLogger(__name__)


class ValidationErrorDetail(BaseModel):
    """Validation error detail model"""
    field: str
    message: str
    value: Any
    constraint: Optional[str] = None


class ValidationResult(BaseModel):
    """Validation result model"""
    is_valid: bool
    errors: List[ValidationErrorDetail] = []
    warnings: List[str] = []


class VelocityValidator:
    """Velocity command validator"""
    
    def __init__(self):
        self.vx_max = settings.VX_MAX
        self.vy_max = settings.VY_MAX
        self.max_level = settings.MAX_LEVEL
        self.min_level = settings.MIN_LEVEL
        self.step_x = settings.step_x
        self.step_y = settings.step_y
        
        logger.info(f"Velocity validator initialized: VX_MAX={self.vx_max}, VY_MAX={self.vy_max}")
    
    def validate_velocity_range(self, vx: float, vy: float) -> ValidationResult:
        """Validate velocity values are within acceptable ranges"""
        result = ValidationResult(is_valid=True)
        
        # Validate vx range
        if abs(vx) > self.vx_max:
            result.is_valid = False
            result.errors.append(ValidationErrorDetail(
                field="vx",
                message=f"Velocity X must be between -{self.vx_max} and {self.vx_max}",
                value=vx,
                constraint=f"abs(vx) <= {self.vx_max}"
            ))
        
        # Validate vy range
        if abs(vy) > self.vy_max:
            result.is_valid = False
            result.errors.append(ValidationErrorDetail(
                field="vy",
                message=f"Velocity Y must be between -{self.vy_max} and {self.vy_max}",
                value=vy,
                constraint=f"abs(vy) <= {self.vy_max}"
            ))
        
        # Add warnings for high velocities
        if abs(vx) > self.vx_max * 0.8:
            result.warnings.append(f"High X velocity: {vx} (>{self.vx_max * 0.8})")
        
        if abs(vy) > self.vy_max * 0.8:
            result.warnings.append(f"High Y velocity: {vy} (>{self.vy_max * 0.8})")
        
        return result
    
    def validate_speed_levels(self, levels: Dict[str, int]) -> ValidationResult:
        """Validate speed levels are within acceptable ranges"""
        result = ValidationResult(is_valid=True)
        
        required_keys = ["up", "down", "left", "right"]
        
        # Check for required keys
        for key in required_keys:
            if key not in levels:
                result.is_valid = False
                result.errors.append(ValidationErrorDetail(
                    field=f"levels.{key}",
                    message=f"Missing required level: {key}",
                    value=None,
                    constraint=f"levels must contain {key}"
                ))
                continue
            
            level = levels[key]
            
            # Validate level range
            if not isinstance(level, (int, float)):
                result.is_valid = False
                result.errors.append(ValidationErrorDetail(
                    field=f"levels.{key}",
                    message=f"Level must be a number",
                    value=level,
                    constraint="type: number"
                ))
                continue
            
            if level < self.min_level or level > self.max_level:
                result.is_valid = False
                result.errors.append(ValidationErrorDetail(
                    field=f"levels.{key}",
                    message=f"Level must be between {self.min_level} and {self.max_level}",
                    value=level,
                    constraint=f"{self.min_level} <= level <= {self.max_level}"
                ))
        
        return result
    
    def validate_velocity_consistency(self, vx: float, vy: float, levels: Dict[str, int]) -> ValidationResult:
        """Validate that velocity values are consistent with speed levels"""
        result = ValidationResult(is_valid=True)
        
        # Calculate expected velocities from levels
        net_x_level = levels.get("up", 0) - levels.get("down", 0)
        net_y_level = levels.get("right", 0) - levels.get("left", 0)
        
        expected_vx = max(-self.max_level, min(self.max_level, net_x_level)) * self.step_x
        expected_vy = max(-self.max_level, min(self.max_level, net_y_level)) * self.step_y
        
        # Allow small tolerance for floating point precision
        tolerance = 0.001
        
        if abs(vx - expected_vx) > tolerance:
            result.warnings.append(
                f"Velocity X ({vx}) doesn't match calculated value from levels ({expected_vx})"
            )
        
        if abs(vy - expected_vy) > tolerance:
            result.warnings.append(
                f"Velocity Y ({vy}) doesn't match calculated value from levels ({expected_vy})"
            )
        
        return result
    
    def validate_command(self, vx: float, vy: float, levels: Optional[Dict[str, int]] = None) -> ValidationResult:
        """Comprehensive validation of velocity command"""
        result = ValidationResult(is_valid=True)
        
        # Validate velocity ranges
        velocity_result = self.validate_velocity_range(vx, vy)
        result.is_valid = result.is_valid and velocity_result.is_valid
        result.errors.extend(velocity_result.errors)
        result.warnings.extend(velocity_result.warnings)
        
        # Validate levels if provided
        if levels is not None:
            levels_result = self.validate_speed_levels(levels)
            result.is_valid = result.is_valid and levels_result.is_valid
            result.errors.extend(levels_result.errors)
            result.warnings.extend(levels_result.warnings)
            
            # Validate consistency between velocity and levels
            if result.is_valid:  # Only check consistency if basic validation passes
                consistency_result = self.validate_velocity_consistency(vx, vy, levels)
                result.warnings.extend(consistency_result.warnings)
        
        return result
    
    def clamp_velocity(self, vx: float, vy: float) -> Tuple[float, float]:
        """Clamp velocity values to safe ranges"""
        clamped_vx = max(-self.vx_max, min(self.vx_max, vx))
        clamped_vy = max(-self.vy_max, min(self.vy_max, vy))
        
        if clamped_vx != vx or clamped_vy != vy:
            logger.warning(f"Velocity clamped: ({vx}, {vy}) -> ({clamped_vx}, {clamped_vy})")
        
        return clamped_vx, clamped_vy


class ValidationService:
    """Validation service for centralized validation logic"""
    
    def __init__(self):
        self.velocity_validator = VelocityValidator()
        logger.info("Validation service initialized")
    
    def validate_velocity_command(self, vx: float, vy: float, levels: Optional[Dict[str, int]] = None) -> ValidationResult:
        """Validate velocity command using velocity validator"""
        return self.velocity_validator.validate_command(vx, vy, levels)
    
    def raise_validation_error(self, result: ValidationResult):
        """Raise ValidationError with validation results"""
        if not result.is_valid:
            error_messages = [f"{error.field}: {error.message}" for error in result.errors]
            raise ValidationError(
                detail="Validation failed",
                validation_errors=error_messages
            )
    
    def log_validation_result(self, result: ValidationResult, command_data: Dict[str, Any]):
        """Log validation results"""
        timestamp = datetime.utcnow().isoformat()
        
        if result.is_valid:
            logger.info(f"Validation passed for command: {command_data}")
            if result.warnings:
                logger.warning(f"Validation warnings: {result.warnings}")
        else:
            logger.error(f"Validation failed for command: {command_data}")
            for error in result.errors:
                logger.error(f"  - {error.field}: {error.message} (value: {error.value})")
    
    def log_successful_command(self, vx: float, vy: float, levels: Optional[Dict[str, int]] = None, source: str = "unknown"):
        """Log successful velocity command"""
        logger.info(f"Velocity command processed successfully: vx={vx}, vy={vy}, levels={levels}, source={source}")
    
    def log_validation_failure(self, vx: float, vy: float, errors: List[str], source: str = "unknown"):
        """Log validation failure"""
        logger.error(f"Velocity command validation failed: vx={vx}, vy={vy}, errors={errors}, source={source}")
    
    def calculate_velocity_from_levels(self, levels: Dict[str, int]) -> Tuple[float, float]:
        """Calculate velocity values from speed levels"""
        net_x_level = levels.get("up", 0) - levels.get("down", 0)
        net_y_level = levels.get("right", 0) - levels.get("left", 0)
        
        # Clamp net levels to valid range
        net_x_level = max(-self.velocity_validator.max_level, min(self.velocity_validator.max_level, net_x_level))
        net_y_level = max(-self.velocity_validator.max_level, min(self.velocity_validator.max_level, net_y_level))
        
        # Calculate velocities
        vx = net_x_level * self.velocity_validator.step_x
        vy = net_y_level * self.velocity_validator.step_y
        
        return vx, vy


# Global validation service instance
validation_service = ValidationService()