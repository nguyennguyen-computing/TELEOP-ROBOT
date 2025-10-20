#!/usr/bin/env python3
"""
Test runner for FastAPI Fleet Server

Simple script to run all tests with proper environment setup.
"""

import os
import sys
import subprocess

def setup_test_environment():
    """Set up test environment variables"""
    test_env = {
        "AUTH_ENABLED": "true",
        "API_KEYS": "test-key-123,admin-key-456",
        "JWT_SECRET": "test-secret-key-for-testing-only",
        "VX_MAX": "2.0",
        "VY_MAX": "1.5",
        "ZENOH_LOCATOR": "tcp/localhost:7447",
        "Z_KEY_CMD_VEL": "test/cmd_vel",
        "FLEET_PORT": "8001",  # Use different port for testing
        "DEBUG": "true",
        "LOG_LEVEL": "DEBUG",
    }
    
    for key, value in test_env.items():
        os.environ[key] = value

def check_test_dependencies():
    """Check if test dependencies are installed"""
    missing_deps = []
    
    try:
        import pytest
    except ImportError:
        missing_deps.append("pytest")
    
    try:
        import httpx
    except ImportError:
        missing_deps.append("httpx")
    
    if missing_deps:
        print("❌ Missing test dependencies:")
        for dep in missing_deps:
            print(f"  - {dep}")
        print("\n🔧 To install dependencies, run:")
        print("  python3 setup.py")
        print("  # or")
        print("  pip install -r requirements.txt")
        return False
    
    return True

def run_tests():
    """Run the test suite"""
    setup_test_environment()
    
    print("🧪 Running FastAPI Fleet Server tests...")
    print("=" * 50)
    
    # Check dependencies first
    if not check_test_dependencies():
        return False
    
    try:
        # Run pytest with verbose output
        result = subprocess.run([
            sys.executable, "-m", "pytest", 
            "tests/", 
            "-v", 
            "--tb=short",
            "--color=yes",
            "--asyncio-mode=auto"
        ], check=True)
        
        print("\n" + "=" * 50)
        print("✅ All tests passed successfully!")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Tests failed with exit code: {e.returncode}")
        print("\n🔧 If you see import errors, try:")
        print("  python3 setup.py")
        return False
    except FileNotFoundError:
        print("❌ pytest not found. Please install dependencies:")
        print("  python3 setup.py")
        return False

if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)