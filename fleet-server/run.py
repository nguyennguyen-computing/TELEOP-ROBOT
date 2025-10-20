#!/usr/bin/env python3
"""
Entry point for Fleet Server

Runs the FastAPI application with uvicorn.
"""

import sys
import os

def check_dependencies():
    """Check if required dependencies are installed"""
    missing_deps = []
    
    try:
        import fastapi
    except ImportError:
        missing_deps.append("fastapi")
    
    try:
        import uvicorn
    except ImportError:
        missing_deps.append("uvicorn")
    
    try:
        import pydantic
    except ImportError:
        missing_deps.append("pydantic")
    
    if missing_deps:
        print("❌ Missing required dependencies:")
        for dep in missing_deps:
            print(f"  - {dep}")
        print("\n🔧 To install dependencies, run:")
        print("  python3 setup.py")
        print("  # or")
        print("  pip install -r requirements.txt")
        sys.exit(1)

def main():
    """Main entry point"""
    # Check dependencies first
    check_dependencies()
    
    # Set development environment if no .env file exists
    if not os.path.exists(".env") and os.path.exists(".env.development"):
        print("📝 Using development configuration (.env.development)")
        os.environ.setdefault("ENV_FILE", ".env.development")
    
    try:
        import uvicorn
        from app.config import settings
        
        print(f"🚀 Starting Fleet Server on {settings.HOST}:{settings.PORT}")
        print(f"🔧 Debug mode: {settings.DEBUG}")
        print(f"🔒 Authentication: {'enabled' if settings.AUTH_ENABLED else 'disabled'}")
        print(f"📚 API docs: http://{settings.HOST}:{settings.PORT}/docs")
        
        uvicorn.run(
            "app.main:app",
            host=settings.HOST,
            port=settings.PORT,
            reload=settings.RELOAD,
            log_level=settings.LOG_LEVEL.lower(),
            access_log=settings.DEBUG,
        )
        
    except Exception as e:
        print(f"❌ Failed to start server: {e}")
        print("\n🔧 Try running setup first:")
        print("  python3 setup.py")
        sys.exit(1)

if __name__ == "__main__":
    main()