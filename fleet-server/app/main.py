"""
Main FastAPI application for Fleet Server

Application factory and configuration setup.
"""

import logging
from contextlib import asynccontextmanager
from datetime import datetime

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import ValidationError

from app.config import settings
from app.utils.logging import setup_logging
from app.models.responses import ErrorResponse
from app.api import api_router
from app.services import zenoh_service

# Setup logging
setup_logging()
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager for startup and shutdown"""
    # Startup
    logger.info("Starting FastAPI Fleet Server...")
    logger.info(f"Configuration: VX_MAX={settings.VX_MAX}, VY_MAX={settings.VY_MAX}")
    logger.info(f"Zenoh locator: {settings.ZENOH_LOCATOR}")
    logger.info(f"Authentication enabled: {settings.AUTH_ENABLED}")
    logger.info(f"Debug mode: {settings.DEBUG}")
    
    # Initialize services
    try:
        await zenoh_service.initialize()
        logger.info("Services initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize services: {e}")
    
    yield
    
    # Shutdown
    logger.info("Shutting down FastAPI Fleet Server...")
    try:
        await zenoh_service.shutdown()
        logger.info("Services shutdown complete")
    except Exception as e:
        logger.error(f"Error during shutdown: {e}")


def create_application() -> FastAPI:
    """Create and configure FastAPI application"""
    
    app = FastAPI(
        title=settings.APP_NAME,
        description=settings.APP_DESCRIPTION,
        version=settings.APP_VERSION,
        docs_url="/docs",
        redoc_url="/redoc",
        lifespan=lifespan,
        debug=settings.DEBUG,
    )
    
    # Configure CORS middleware
    cors_config = settings.get_cors_config()
    app.add_middleware(
        CORSMiddleware,
        allow_origins=cors_config["allow_origins"],
        allow_credentials=cors_config["allow_credentials"],
        allow_methods=cors_config["allow_methods"],
        allow_headers=cors_config["allow_headers"],
    )
    
    # Add custom exception handlers
    setup_exception_handlers(app)
    
    # Include API router
    app.include_router(api_router, prefix="/api/v1")
    
    # Include health endpoints at root level for convenience
    from app.api.v1.endpoints.health import router as health_router
    app.include_router(health_router)
    
    return app


def setup_exception_handlers(app: FastAPI):
    """Setup custom exception handlers"""
    
    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        """Handle HTTP exceptions"""
        return JSONResponse(
            status_code=exc.status_code,
            content=ErrorResponse(
                error=str(exc.detail),
                timestamp=datetime.utcnow().isoformat()
            ).dict()
        )
    
    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        """Handle request validation errors"""
        errors = []
        for error in exc.errors():
            field = " -> ".join(str(loc) for loc in error["loc"])
            errors.append(f"{field}: {error['msg']}")
        
        return JSONResponse(
            status_code=422,
            content=ErrorResponse(
                error="Request Validation Error",
                detail="Invalid request data",
                errors=errors,
                timestamp=datetime.utcnow().isoformat()
            ).dict()
        )
    
    @app.exception_handler(ValidationError)
    async def pydantic_validation_exception_handler(request: Request, exc: ValidationError):
        """Handle Pydantic validation errors"""
        errors = []
        for error in exc.errors():
            field = " -> ".join(str(loc) for loc in error["loc"])
            errors.append(f"{field}: {error['msg']}")
        
        return JSONResponse(
            status_code=400,
            content=ErrorResponse(
                error="Validation Error",
                detail="Data validation failed",
                errors=errors,
                timestamp=datetime.utcnow().isoformat()
            ).dict()
        )
    
    @app.exception_handler(ValueError)
    async def value_error_handler(request: Request, exc: ValueError):
        """Handle value errors"""
        return JSONResponse(
            status_code=400,
            content=ErrorResponse(
                error="Value Error",
                detail=str(exc),
                timestamp=datetime.utcnow().isoformat()
            ).dict()
        )
    
    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        """Handle unexpected exceptions"""
        logger.error(f"Unexpected error: {exc}", exc_info=True)
        
        # Don't expose internal errors in production
        detail = str(exc) if settings.DEBUG else "Internal server error"
        
        return JSONResponse(
            status_code=500,
            content=ErrorResponse(
                error="Internal Server Error",
                detail=detail,
                timestamp=datetime.utcnow().isoformat()
            ).dict()
        )


# Create application instance
app = create_application()