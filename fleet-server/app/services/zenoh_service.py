"""
Zenoh service for Fleet Server - Fixed for Bridge Communication
"""

import json
import asyncio
import logging
from typing import Optional, Dict, Any
from datetime import datetime
from contextlib import asynccontextmanager

from app.config import settings
from app.core.exceptions import ZenohConnectionError

# Try to import Zenoh
try:
    import zenoh
    ZENOH_AVAILABLE = True
except ImportError:
    try:
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
        return {
            "linear": self.linear,
            "angular": self.angular
        }
    
    @classmethod
    def from_velocity(cls, vx: float, vy: float) -> 'TwistMessage':
        return cls(vx=vx, vy=vy, vz=0.0, ax=0.0, ay=0.0, az=0.0)


class ZenohPublisher:
    """Zenoh publisher for robot commands"""
    
    def __init__(self):
        self.config = settings.get_zenoh_config()
        self.session: Optional[Any] = None
        self.is_connected = False
        self.reconnect_attempts = 0
        self.last_publish_time: Optional[datetime] = None
        self.publish_count = 0
        self.zenoh_available = ZENOH_AVAILABLE
        
        # CRITICAL FIX: Ensure topic key is in ROS2 format
        self._fix_topic_key()
        
        if not self.zenoh_available:
            logger.warning("Zenoh not available - running in mock mode")
        else:
            logger.info(f"ZenohPublisher initialized")
            logger.info(f"Zenoh locator: {self.config['locator']}")
            logger.info(f"Command velocity key: {self.config['cmd_vel_key']}")
    
    def _fix_topic_key(self):
        """Ensure topic key is in correct ROS2 format"""
        cmd_vel_key = self.config.get('cmd_vel_key', 'cmd_vel')
        
        # Remove leading/trailing slashes if any
        cmd_vel_key = cmd_vel_key.strip('/')
        
        self.config['cmd_vel_key'] = cmd_vel_key
        logger.info(f"Topic key normalized to: {cmd_vel_key}")
    
    async def connect(self) -> bool:
        """Connect to Zenoh router"""
        if not self.zenoh_available:
            logger.info("Zenoh not available - using mock connection")
            self.is_connected = True
            return True
            
        try:
            logger.info(f"Connecting to Zenoh router at {self.config['locator']}")
            
            # Parse locator
            locator = self.config['locator']
            
            # Create Zenoh configuration
            zenoh_config = zenoh.Config()
            
            # Set connection endpoint
            endpoints = [locator] if not locator.startswith('[') else json.loads(locator)
            zenoh_config.insert_json5("connect/endpoints", json.dumps(endpoints))
            
            # Set mode to client (important for connecting to router)
            zenoh_config.insert_json5("mode", '"client"')
            
            # Open session
            logger.info(f"Opening Zenoh session with config: {zenoh_config}")
            self.session = zenoh.open(zenoh_config)
            
            if self.session:
                self.is_connected = True
                self.reconnect_attempts = 0
                logger.info("✅ Successfully connected to Zenoh router")
                logger.info(f"Session info: {type(self.session)}")
                return True
            else:
                logger.error("Failed to create Zenoh session")
                return False
                
        except Exception as e:
            logger.error(f"Failed to connect to Zenoh: {e}", exc_info=True)
            # Don't fallback to mock mode in production
            self.is_connected = False
            return False
    
    async def disconnect(self):
        """Disconnect from Zenoh router"""
        if self.session:
            try:
                self.session.close()
                logger.info("Disconnected from Zenoh router")
            except Exception as e:
                logger.error(f"Error during disconnect: {e}")
            finally:
                self.session = None
                self.is_connected = False
    
    def _create_twist_cdr(self, vx: float, vy: float) -> bytes:
        """
        Create CDR (Common Data Representation) binary for geometry_msgs/Twist
        
        Twist message structure:
        - geometry_msgs/Vector3 linear  (x, y, z as float64)
        - geometry_msgs/Vector3 angular (x, y, z as float64)
        
        CDR encapsulation format:
        - 4 bytes header: [0x00, 0x01, 0x00, 0x00] (little endian, CDRv1)
        - 48 bytes data: 6 x float64 (8 bytes each)
        """
        import struct
        
        # Round to reasonable precision
        vx_val = round(float(vx), 3)
        vy_val = round(float(vy), 3)
        
        logger.debug(f"Creating CDR for vx={vx_val}, vy={vy_val}")
        
        # CDR encapsulation header
        # Byte 0: Endianness (0x00 = big endian, 0x01 = little endian)
        # Byte 1: Encapsulation kind (0x00 = CDR_BE, 0x01 = CDR_LE)
        cdr_header = bytes([0x00, 0x01, 0x00, 0x00])
        
        # Linear velocity (Vector3): x, y, z (float64 = double)
        linear_x = struct.pack('<d', vx_val)  # '<' = little endian, 'd' = double
        linear_y = struct.pack('<d', vy_val)
        linear_z = struct.pack('<d', 0.0)
        
        # Angular velocity (Vector3): x, y, z (float64 = double)
        angular_x = struct.pack('<d', 0.0)
        angular_y = struct.pack('<d', 0.0)
        angular_z = struct.pack('<d', 0.0)
        
        # Combine all
        cdr_data = (cdr_header + 
                   linear_x + linear_y + linear_z + 
                   angular_x + angular_y + angular_z)
        
        logger.debug(f"CDR data created: {len(cdr_data)} bytes")
        logger.debug(f"CDR hex: {cdr_data.hex()}")
        
        return cdr_data

    async def publish_twist(self, vx: float, vy: float, 
                           metadata: Optional[Dict[str, Any]] = None) -> bool:
        """Publish Twist message via Zenoh"""
        try:
            # Check connection
            if not self.is_connected or not self.session:
                logger.error("Cannot publish: Not connected to Zenoh")
                return False
            
            # Create CDR message
            message_data = self._create_twist_cdr(vx, vy)
            topic_key = self.config['cmd_vel_key']
            
            # Publish via Zenoh
            logger.info(f"Publishing to Zenoh key: {topic_key}")
            logger.debug(f"Message size: {len(message_data)} bytes")
            logger.debug(f"Velocity: vx={vx}, vy={vy}")
            
            # Use put() to publish
            self.session.put(topic_key, message_data)
            
            # Update stats
            self.last_publish_time = datetime.utcnow()
            self.publish_count += 1
            
            logger.info(f"✅ Published Twist #{self.publish_count} to {topic_key}")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to publish Twist: {e}", exc_info=True)
            self.is_connected = False
            return False
    
    async def publish_velocity_command(self, vx: float, vy: float, 
                                     levels: Optional[Dict[str, int]] = None,
                                     source: str = "fleet") -> bool:
        """Publish velocity command with metadata"""
        metadata = {
            "timestamp": datetime.utcnow().isoformat(),
            "source": source,
            "publish_count": self.publish_count + 1
        }
        
        if levels:
            metadata["levels"] = levels
        
        return await self.publish_twist(vx, vy, metadata)
    
    def get_status(self) -> Dict[str, Any]:
        """Get publisher status"""
        return {
            "connected": self.is_connected,
            "zenoh_available": self.zenoh_available,
            "locator": self.config['locator'],
            "cmd_vel_key": self.config['cmd_vel_key'],
            "reconnect_attempts": self.reconnect_attempts,
            "last_publish_time": self.last_publish_time.isoformat() if self.last_publish_time else None,
            "publish_count": self.publish_count,
            "session_active": self.session is not None,
            "mode": "production" if self.zenoh_available else "mock"
        }
    
    async def health_check(self) -> bool:
        """Health check"""
        return self.is_connected and self.session is not None


class ZenohService:
    """Zenoh service for centralized operations"""
    
    def __init__(self):
        self.publisher: Optional[ZenohPublisher] = None
        logger.info("Zenoh service initialized")
    
    async def initialize(self) -> bool:
        """Initialize publisher"""
        try:
            self.publisher = ZenohPublisher()
            connected = await self.publisher.connect()
            
            if connected:
                logger.info("✅ Zenoh service ready")
                return True
            else:
                logger.error("❌ Zenoh service initialization failed")
                return False
                
        except Exception as e:
            logger.error(f"Failed to initialize Zenoh: {e}", exc_info=True)
            return False
    
    async def shutdown(self):
        """Shutdown service"""
        try:
            if self.publisher:
                await self.publisher.disconnect()
                self.publisher = None
            logger.info("Zenoh service shutdown complete")
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
    
    async def publish_velocity(self, vx: float, vy: float, 
                              levels: Optional[Dict[str, int]] = None,
                              source: str = "fleet") -> bool:
        """Publish velocity command"""
        if not self.publisher:
            logger.error("Publisher not initialized")
            return False
        
        return await self.publisher.publish_velocity_command(vx, vy, levels, source)
    
    def get_status(self) -> Dict[str, Any]:
        """Get service status"""
        if self.publisher:
            return self.publisher.get_status()
        return {"connected": False, "error": "Not initialized"}
    
    async def health_check(self) -> bool:
        """Health check"""
        if self.publisher:
            return await self.publisher.health_check()
        return False


# Global instance
zenoh_service = ZenohService()