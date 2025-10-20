#!/usr/bin/env python3
"""
Quick startup test for Fleet Server
"""

import os
import sys
import asyncio
import signal
import time
from multiprocessing import Process

# Set test environment
os.environ.update({
    'DEBUG': 'true',
    'AUTH_ENABLED': 'false',
    'JWT_SECRET': 'test-secret',
    'VX_MAX': '1.0',
    'VY_MAX': '1.5',
    'FLEET_PORT': '8001'  # Use different port for testing
})

def run_server():
    """Run the server in a separate process"""
    try:
        import uvicorn
        from app.main import app
        
        uvicorn.run(
            app,
            host="127.0.0.1",
            port=8001,
            log_level="error"  # Reduce log noise
        )
    except Exception as e:
        print(f"Server error: {e}")

def test_startup():
    """Test that the server can start"""
    print("🧪 Testing Fleet Server startup...")
    
    # Start server in background
    server_process = Process(target=run_server)
    server_process.start()
    
    try:
        # Give server time to start
        time.sleep(3)
        
        # Check if server is still running
        if server_process.is_alive():
            print("✅ Server started successfully!")
            print("🌐 Server running on http://127.0.0.1:8001")
            
            # Test basic import
            try:
                import httpx
                response = httpx.get("http://127.0.0.1:8001/health", timeout=5)
                if response.status_code == 200:
                    print("✅ Health check passed!")
                else:
                    print(f"⚠️  Health check returned: {response.status_code}")
            except Exception as e:
                print(f"⚠️  Health check failed: {e}")
            
            result = True
        else:
            print("❌ Server failed to start")
            result = False
            
    finally:
        # Clean up
        if server_process.is_alive():
            server_process.terminate()
            server_process.join(timeout=5)
            if server_process.is_alive():
                server_process.kill()
    
    return result

if __name__ == "__main__":
    success = test_startup()
    
    if success:
        print("\n🎉 Fleet Server startup test PASSED!")
        print("🚀 Ready to run: python3 run.py")
        sys.exit(0)
    else:
        print("\n❌ Fleet Server startup test FAILED!")
        sys.exit(1)