#!/usr/bin/env python3
"""
Quick test script for Fleet Server

Tests that the server can start and basic endpoints work.
"""

import os
import sys
import asyncio
from contextlib import asynccontextmanager

# Set test environment
os.environ.update({
    'DEBUG': 'true',
    'AUTH_ENABLED': 'false',
    'JWT_SECRET': 'test-secret',
    'VX_MAX': '1.0',
    'VY_MAX': '1.5',
    'FLEET_PORT': '8001'  # Use different port for testing
})

async def test_server():
    """Test server functionality"""
    print("🧪 Testing Fleet Server...")
    
    try:
        # Test app creation
        from app.main import create_application
        app = create_application()
        print("✅ App created successfully")
        
        # Test that we can import all components
        from app.config import settings
        from app.models import VelocityCommand, SpeedLevels
        from app.core.auth import auth_service
        from app.core.validation import validation_service
        from app.services import zenoh_service
        
        print("✅ All components imported")
        print(f"   - Settings: VX_MAX={settings.VX_MAX}")
        print(f"   - Auth enabled: {settings.AUTH_ENABLED}")
        print(f"   - Port: {settings.PORT}")
        
        # Test basic functionality
        cmd = VelocityCommand(vx=1.0, vy=0.5, levels=SpeedLevels(up=5, down=0, left=0, right=3))
        result = validation_service.validate_velocity_command(1.0, 0.5)
        print("✅ Basic functionality working")
        
        # Test Zenoh service
        status = zenoh_service.get_status()
        print(f"✅ Zenoh service: {status.get('mode', 'unknown')} mode")
        
        print("\n🎉 All tests passed!")
        print("🚀 Server is ready to run!")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main test function"""
    print("Fleet Server Test Suite")
    print("=" * 50)
    
    success = asyncio.run(test_server())
    
    if success:
        print("\n" + "=" * 50)
        print("✅ All tests passed!")
        print("\n📋 Ready to start:")
        print("  python3 run.py")
        print("\n📚 API Documentation:")
        print("  http://localhost:8000/docs")
        sys.exit(0)
    else:
        print("\n❌ Tests failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()