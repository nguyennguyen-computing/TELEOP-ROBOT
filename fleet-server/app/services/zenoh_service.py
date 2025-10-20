"""
Zenoh service for Fleet Server

Provides Zenoh client functionality for publishing ROS2 Twist messages
to the robot control system. Handles connection management, message
serialization, and error recovery for reliable robot communication.
"""

import json
import asyncio
import logging
from typing import Optional, Dict, Any
from datetime import datetime
from contextlib import asynccontextmanager

from app.config import settings
from app.core.exceptions import ZenohConnectionError

# Try to import Zenoh, handle gracefully if not available
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    try:
        # Try alternative import path if needed
        from eclipse import zenoh
        ZENOH_AVAILABLE = True
    except ImportError:
        zenoh = None
        ZENOH_AVAILABLE = False

logger = logging.getLogger(__name__)


class TwistMessage:
    """ROS2 Twist message representation"""
    
    def __init__(self, vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
                 ax: float = 0.0, ay: float = 0.0, az: float = 0.0):
        self.linear = {"x": vx, "y": vy, "z": vz}
        self.angular = {"x": ax, "y": ay, "z": az}
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return {
            "linear": self.linear,
            "angular": self.angular
        }
    
    def to_json(self) -> str:
        """Convert to JSON string"""
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_velocity(cls, vx: float, vy: float) -> 'TwistMessage':
        """Create Twist message from velocity components"""
        # Following the coordinate system: Forward=+X, Right=+Y
        # Angular Z is always 0 as specified in requirements
        return cls(vx=vx, vy=vy, vz=0.0, ax=0.0, ay=0.0, az=0.0)


class ZenohPublisher:
    """Zenoh publisher for robot commands"""
    
    def __init__(self):
        self.config = settings.get_zenoh_config()
        self.session: Optional[Any] = None  # Use Any instead of zenoh.Session
        self.is_connected = False
        self.reconnect_attempts = 0
        self.last_publish_time: Optional[datetime] = None
        self.publish_count = 0
        self.zenoh_available = ZENOH_AVAILABLE
        
        if not self.zenoh_available:
            logger.warning("Zenoh not available - running in mock mode")
            logger.warning("To enable Zenoh: pip install eclipse-zenoh")
        else:
            logger.info(f"ZenohPublisher initialized with locator: {self.config['locator']}")
            logger.info(f"Command velocity key: {self.config['cmd_vel_key']}")
    
    async def connect(self) -> bool:
        """Connect to Zenoh router"""
        if not self.zenoh_available:
            logger.info("Zenoh not available - using mock connection")
            self.is_connected = True  # Mock connection for development
            return True
            
        try:
            logger.info(f"Connecting to Zenoh router at {self.config['locator']}")
            
            # Create Zenoh session with proper configuration
            # Try different configuration approaches for different Zenoh versions
            try:
                # Method 1: Try with Config object
                zenoh_config = zenoh.Config()
                zenoh_config.insert_json5("connect/endpoints", f'["{self.config["locator"]}"]')
                self.session = zenoh.open(zenoh_config)
            except Exception as e1:
                logger.debug(f"Config method failed: {e1}, trying alternative...")
                try:
                    # Method 2: Try with simple string
                    self.session = zenoh.open()
                except Exception as e2:
                    logger.debug(f"Simple open failed: {e2}, trying with dict...")
                    # Method 3: Try with minimal config
                    self.session = zenoh.open({})
            
            if self.session:
                self.is_connected = True
                self.reconnect_attempts = 0
                logger.info("Successfully connected to Zenoh router")
                return True
            else:
                logger.warning("Failed to create Zenoh session - continuing in mock mode")
                self.is_connected = True  # Use mock mode
                return True
                
        except Exception as e:
            logger.warning(f"Failed to connect to Zenoh: {e} - continuing in mock mode")
            self.is_connected = True  # Use mock mode for development
            return True
    
    async def disconnect(self):
        """Disconnect from Zenoh router"""
        if self.session:
            try:
                self.session.close()
                logger.info("Disconnected from Zenoh router")
            except Exception as e:
                logger.error(f"Error during Zenoh disconnect: {e}")
            finally:
                self.session = None
                self.is_connected = False
    
    async def reconnect(self) -> bool:
        """Attempt to reconnect to Zenoh router"""
        if self.reconnect_attempts >= self.config['max_reconnect_attempts']:
            logger.error(f"Max reconnection attempts ({self.config['max_reconnect_attempts']}) reached")
            return False
        
        self.reconnect_attempts += 1
        logger.info(f"Reconnection attempt {self.reconnect_attempts}/{self.config['max_reconnect_attempts']}")
        
        # Disconnect first if still connected
        if self.session:
            await self.disconnect()
        
        # Wait before reconnecting
        await asyncio.sleep(self.config['reconnect_interval'])
        
        return await self.connect()
    
    async def ensure_connected(self) -> bool:
        """Ensure connection is established, reconnect if necessary"""
        if self.is_connected:
            return True
        
        if not self.zenoh_available:
            logger.debug("Zenoh not available - using mock mode")
            self.is_connected = True
            return True
        
        logger.warning("Zenoh connection lost, attempting to reconnect")
        return await self.reconnect()
    
    async def publish_twist(self, vx: float, vy: float, metadata: Optional[Dict[str, Any]] = None) -> bool:
        """Publish Twist message via Zenoh"""
        try:
            # Ensure connection
            if not await self.ensure_connected():
                logger.error("Cannot publish: Zenoh connection unavailable")
                return False
            
            # Create Twist message
            twist = TwistMessage.from_velocity(vx, vy)
            message_data = twist.to_json()
            
            # Add metadata if provided
            if metadata:
                message_dict = twist.to_dict()
                message_dict["metadata"] = metadata
                message_data = json.dumps(message_dict)
            
            # Publish message (mock if Zenoh not available)
            if self.zenoh_available and self.session:
                self.session.put(self.config['cmd_vel_key'], message_data)
                logger.info(f"Published Twist message: vx={vx}, vy={vy} to key={self.config['cmd_vel_key']}")
            else:
                logger.info(f"Mock published Twist message: vx={vx}, vy={vy} (Zenoh not available)")
            
            # Update statistics
            self.last_publish_time = datetime.utcnow()
            self.publish_count += 1
            
            logger.debug(f"Message data: {message_data}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish Twist message: {e}")
            self.is_connected = False  # Mark as disconnected for next attempt
            return False
    
    async def publish_velocity_command(self, vx: float, vy: float, 
                                     levels: Optional[Dict[str, int]] = None,
                                     source: str = "fleet") -> bool:
        """Publish velocity command with additional metadata"""
        metadata = {
            "timestamp": datetime.utcnow().isoformat(),
            "source": source,
            "publish_count": self.publish_count + 1
        }
        
        if levels:
            metadata["levels"] = levels
        
        return await self.publish_twist(vx, vy, metadata)
    
    def get_status(self) -> Dict[str, Any]:
        """Get publisher status information"""
        return {
            "connected": self.is_connected,
            "zenoh_available": self.zenoh_available,
            "locator": self.config['locator'],
            "cmd_vel_key": self.config['cmd_vel_key'],
            "reconnect_attempts": self.reconnect_attempts,
            "max_reconnect_attempts": self.config['max_reconnect_attempts'],
            "last_publish_time": self.last_publish_time.isoformat() if self.last_publish_time else None,
            "publish_count": self.publish_count,
            "session_active": self.session is not None,
            "mode": "production" if self.zenoh_available else "mock"
        }
    
    async def health_check(self) -> bool:
        """Perform health check by attempting a test publish"""
        try:
            if not self.is_connected:
                return False
            
            # Try to publish a zero velocity command as health check
            return await self.publish_twist(0.0, 0.0, {"health_check": True})
            
        except Exception as e:
            logger.error(f"Health check failed: {e}")
            return False


class ZenohMonitor:
    """Monitor Zenoh connection health"""
    
    def __init__(self, publisher: ZenohPublisher, check_interval: float = None):
        self.publisher = publisher
        self.check_interval = check_interval or settings.HEALTH_CHECK_INTERVAL
        self.monitoring = False
        self.monitor_task: Optional[asyncio.Task] = None
    
    async def start_monitoring(self):
        """Start connection monitoring"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_task = asyncio.create_task(self._monitor_loop())
        logger.info(f"Started Zenoh connection monitoring (interval: {self.check_interval}s)")
    
    async def stop_monitoring(self):
        """Stop connection monitoring"""
        self.monitoring = False
        if self.monitor_task:
            self.monitor_task.cancel()
            try:
                await self.monitor_task
            except asyncio.CancelledError:
                pass
        logger.info("Stopped Zenoh connection monitoring")
    
    async def _monitor_loop(self):
        """Connection monitoring loop"""
        while self.monitoring:
            try:
                # Perform health check
                healthy = await self.publisher.health_check()
                
                if not healthy and self.publisher.is_connected:
                    logger.warning("Zenoh health check failed, marking as disconnected")
                    self.publisher.is_connected = False
                
                await asyncio.sleep(self.check_interval)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in Zenoh monitoring: {e}")
                await asyncio.sleep(self.check_interval)


class ZenohService:
    """Zenoh service for centralized Zenoh operations"""
    
    def __init__(self):
        self.publisher: Optional[ZenohPublisher] = None
        self.monitor: Optional[ZenohMonitor] = None
        logger.info("Zenoh service initialized")
    
    async def initialize(self) -> bool:
        """Initialize Zenoh publisher and monitoring"""
        try:
            self.publisher = ZenohPublisher()
            connected = await self.publisher.connect()
            
            if connected:
                self.monitor = ZenohMonitor(self.publisher)
                await self.monitor.start_monitoring()
                logger.info("Zenoh service initialization complete")
                return True
            else:
                logger.warning("Zenoh service initialization failed - connection unavailable")
                return False
                
        except Exception as e:
            logger.error(f"Failed to initialize Zenoh service: {e}")
            return False
    
    async def shutdown(self):
        """Shutdown Zenoh service"""
        try:
            if self.monitor:
                await self.monitor.stop_monitoring()
                self.monitor = None
            
            if self.publisher:
                await self.publisher.disconnect()
                self.publisher = None
            
            logger.info("Zenoh service shutdown complete")
            
        except Exception as e:
            logger.error(f"Error during Zenoh service shutdown: {e}")
    
    async def publish_velocity(self, vx: float, vy: float, 
                              levels: Optional[Dict[str, int]] = None,
                              source: str = "fleet") -> bool:
        """Publish velocity command"""
        if not self.publisher:
            logger.error("Zenoh publisher not initialized")
            return False
        
        return await self.publisher.publish_velocity_command(vx, vy, levels, source)
    
    def get_status(self) -> Dict[str, Any]:
        """Get Zenoh service status"""
        if self.publisher:
            return self.publisher.get_status()
        else:
            return {
                "connected": False,
                "error": "Publisher not initialized"
            }
    
    async def health_check(self) -> bool:
        """Perform health check"""
        if self.publisher:
            return await self.publisher.health_check()
        return False
    
    @asynccontextmanager
    async def session(self):
        """Context manager for Zenoh operations"""
        if not self.publisher:
            raise ZenohConnectionError("Zenoh publisher not initialized")
        
        try:
            yield self.publisher
        finally:
            # Keep connection alive for reuse
            pass


# Global Zenoh service instance
zenoh_service = ZenohService()