"""
Fleet control endpoints for Fleet Server API

Provides robot velocity control and command endpoints.
"""

import logging
from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, status

from app.models.requests import VelocityCommand, SpeedLevels, StopCommand
from app.models.responses import VelocityResponse
from app.core.auth import AuthUser
from app.core.validation import ValidationService
from app.services import ZenohService
from app.core.auth import RequireRobotControl
from app.api.deps import get_validation_service, get_zenoh_service

logger = logging.getLogger(__name__)

router = APIRouter()


@router.post("/vel", response_model=VelocityResponse, tags=["Fleet Control"])
async def set_velocity(
    command: VelocityCommand,
    user: AuthUser = RequireRobotControl,
    validation_svc: ValidationService = Depends(get_validation_service),
    zenoh_svc: ZenohService = Depends(get_zenoh_service)
):
    """Set robot velocity via Zenoh messaging"""
    
    try:
        # Validate velocity command
        validation_result = validation_svc.validate_velocity_command(
            command.vx, 
            command.vy, 
            command.levels.dict() if command.levels else None
        )
        
        # Log validation result
        validation_svc.log_validation_result(validation_result, command.dict())
        
        # Raise error if validation fails
        if not validation_result.is_valid:
            validation_svc.log_validation_failure(
                command.vx, 
                command.vy, 
                [error.message for error in validation_result.errors],
                user.id
            )
            validation_svc.raise_validation_error(validation_result)
        
        # Publish velocity command via Zenoh
        success = await zenoh_svc.publish_velocity(
            command.vx,
            command.vy,
            command.levels.dict() if command.levels else None,
            user.id
        )
        
        if success:
            validation_svc.log_successful_command(
                command.vx,
                command.vy,
                command.levels.dict() if command.levels else None,
                user.id
            )
            
            return VelocityResponse(
                published=True,
                message="Velocity command published successfully",
                timestamp=datetime.utcnow().isoformat(),
                command=command
            )
        else:
            logger.error(f"Failed to publish velocity command: {command.dict()}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Failed to publish command to robot"
            )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in set_velocity: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}"
        )


@router.post("/stop", response_model=VelocityResponse, tags=["Fleet Control"])
async def stop_robot(
    stop_cmd: StopCommand = StopCommand(),
    user: AuthUser = RequireRobotControl,
    validation_svc: ValidationService = Depends(get_validation_service),
    zenoh_svc: ZenohService = Depends(get_zenoh_service)
):
    """Stop robot by sending zero velocity"""
    
    try:
        # Create stop command
        stop_command = VelocityCommand(
            vx=0.0,
            vy=0.0,
            levels=SpeedLevels(up=0, down=0, left=0, right=0),
            source=stop_cmd.source or user.id
        )
        
        # Publish stop command
        success = await zenoh_svc.publish_velocity(
            0.0, 
            0.0, 
            stop_command.levels.dict(), 
            stop_command.source
        )
        
        if success:
            validation_svc.log_successful_command(
                0.0, 
                0.0, 
                stop_command.levels.dict(), 
                stop_command.source
            )
            
            return VelocityResponse(
                published=True,
                message="Stop command published successfully",
                timestamp=datetime.utcnow().isoformat(),
                command=stop_command
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="Failed to publish stop command to robot"
            )
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in stop_robot: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}"
        )