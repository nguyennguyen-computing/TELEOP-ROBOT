#!/usr/bin/env python3
"""
Setup script for Fleet Server

Installs dependencies and sets up the development environment.
"""

import os
import sys
import subprocess
import platform

def install_dependencies():
    """Install Python dependencies"""
    print("📦 Installing Python dependencies...")
    
    try:
        # Upgrade pip first
        subprocess.run([sys.executable, "-m", "pip", "install", "--upgrade", "pip"], check=True)
        
        # Install all requirements (including Zenoh with correct package name)
        subprocess.run([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"], check=True)
        
        print("✅ All dependencies installed successfully!")
        
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install dependencies: {e}")
        return False
    except FileNotFoundError:
        print("❌ pip not found. Please install Python and pip first.")
        return False

def setup_environment():
    """Set up environment variables"""
    print("⚙️ Setting up environment...")
    
    env_file = ".env"
    if not os.path.exists(env_file):
        print(f"📝 Creating {env_file} file...")
        
        env_content = """# Fleet Server Configuration

# Server Configuration
FLEET_HOST=0.0.0.0
FLEET_PORT=8000
DEBUG=true
RELOAD=true

# Velocity Limits
VX_MAX=1.0
VY_MAX=1.0

# Zenoh Configuration
ZENOH_LOCATOR=tcp/zenoh:7447
Z_KEY_CMD_VEL=rt/ros2/cmd_vel

# Authentication (disabled by default for development)
AUTH_ENABLED=false
API_KEYS=dev-key-123,admin-key-456
JWT_SECRET=dev-secret-for-testing-only
JWT_EXPIRES_IN=24h

# CORS
CORS_ORIGINS=http://localhost:3000,http://localhost:3001

# Logging
LOG_LEVEL=INFO
"""
        
        with open(env_file, "w") as f:
            f.write(env_content)
        
        print(f"✅ Created {env_file} with default configuration")
    else:
        print(f"✅ {env_file} already exists")

def test_installation():
    """Test the installation"""
    print("🧪 Testing installation...")
    
    # Set test environment
    os.environ.update({
        "DEBUG": "true",
        "AUTH_ENABLED": "false",
        "JWT_SECRET": "test-secret",
        "VX_MAX": "1.0",
        "VY_MAX": "1.0"
    })
    
    try:
        # Test imports
        from app.config import settings
        from app.models import VelocityCommand, SpeedLevels
        from app.core.auth import auth_service
        from app.core.validation import validation_service
        from app.services import zenoh_service
        
        print("✅ All imports successful")
        
        # Test basic functionality
        cmd = VelocityCommand(vx=1.0, vy=0.5, levels=SpeedLevels(up=5, down=0, left=0, right=3))
        result = validation_service.validate_velocity_command(1.0, 0.5)
        
        print("✅ Basic functionality working")
        print(f"✅ Configuration loaded: VX_MAX={settings.VX_MAX}")
        
        # Check Zenoh availability
        try:
            import zenoh
            print("✅ Zenoh available - full functionality enabled")
        except ImportError:
            print("⚠️  Zenoh not available - running in mock mode")
        
        return True
        
    except Exception as e:
        print(f"❌ Installation test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main setup function"""
    print("🚀 Fleet Server Setup")
    print("=" * 50)
    
    # Check Python version
    python_version = platform.python_version()
    print(f"🐍 Python version: {python_version}")
    
    if sys.version_info < (3, 8):
        print("❌ Python 3.8 or higher is required")
        sys.exit(1)
    
    # Install dependencies
    if not install_dependencies():
        sys.exit(1)
    
    # Setup environment
    setup_environment()
    
    # Test installation
    if not test_installation():
        print("\n❌ Setup failed. Please check the error messages above.")
        sys.exit(1)
    
    print("\n" + "=" * 50)
    print("🎉 Setup completed successfully!")
    print("\n📋 Next steps:")
    print("  1. Review .env file for configuration")
    print("  2. Run the server: python3 run.py")
    print("  3. Run tests: python3 run_tests.py")
    print("  4. View API docs: http://localhost:8000/docs")
    print("\n🔧 Development commands:")
    print("  • Start server: python3 run.py")
    print("  • Run tests: python3 run_tests.py")
    print("  • Install deps: pip install -r requirements.txt")
    print("\n📡 Zenoh Integration:")
    print("  • Zenoh included in requirements (eclipse-zenoh)")
    print("  • Server auto-detects Zenoh availability")
    print("  • Check /status endpoint for connection status")

if __name__ == "__main__":
    main()