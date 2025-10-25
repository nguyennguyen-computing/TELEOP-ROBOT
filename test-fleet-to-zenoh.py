#!/usr/bin/env python3
"""
Test Fleet API -> Zenoh -> Bridge -> Robot pipeline
"""
import asyncio
import zenoh
import json
import requests
import time

async def monitor_zenoh_and_test_fleet():
    """Monitor Zenoh while testing Fleet API"""
    print("🔍 Testing Fleet API -> Zenoh -> Bridge -> Robot")
    print("=" * 50)
    
    # Connect to Zenoh
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps(["tcp/localhost:7447"]))
    session = zenoh.open(config)
    
    received_messages = []
    
    def callback(sample):
        timestamp = time.strftime("%H:%M:%S")
        message = {
            "timestamp": timestamp,
            "topic": str(sample.key_expr),
            "payload_size": len(sample.payload)
        }
        received_messages.append(message)
        print(f"[{timestamp}] 📨 Zenoh received: {sample.key_expr} ({len(sample.payload)} bytes)")
        
        # Try to decode CDR if it's cmd_vel
        if "cmd_vel" in str(sample.key_expr):
            try:
                import struct
                payload_bytes = bytes(sample.payload)
                if len(payload_bytes) >= 52:  # CDR + 6 doubles
                    data = payload_bytes[4:]  # Skip CDR header
                    values = struct.unpack('<6d', data[:48])
                    print(f"    Twist: linear=({values[0]:.2f}, {values[1]:.2f}, {values[2]:.2f})")
                    print(f"           angular=({values[3]:.2f}, {values[4]:.2f}, {values[5]:.2f})")
            except Exception as e:
                print(f"    Decode error: {e}")
    
    # Subscribe to all topics
    subscriber = session.declare_subscriber("**", callback)
    print("👂 Monitoring Zenoh topics...")
    
    # Wait a bit for monitor to start
    await asyncio.sleep(2)
    
    # Test Fleet API
    print("\n📤 Testing Fleet API...")
    for i in range(3):
        vx, vy = 0.1 + i * 0.1, 0.05 + i * 0.05
        
        try:
            response = requests.post(
                "http://localhost:8000/api/v1/fleet/vel",
                json={"vx": vx, "vy": vy},
                headers={"Content-Type": "application/json"},
                timeout=5
            )
            
            if response.status_code == 200:
                print(f"✅ Fleet API call {i+1}: vx={vx}, vy={vy}")
            else:
                print(f"❌ Fleet API call {i+1} failed: {response.status_code}")
                print(f"   Response: {response.text}")
                
        except Exception as e:
            print(f"❌ Fleet API call {i+1} error: {e}")
        
        await asyncio.sleep(2)
    
    # Wait a bit more for any delayed messages
    await asyncio.sleep(3)
    
    subscriber.undeclare()
    session.close()
    
    print(f"\n📊 Results:")
    print(f"Messages received on Zenoh: {len(received_messages)}")
    for msg in received_messages:
        print(f"  - {msg['timestamp']}: {msg['topic']} ({msg['payload_size']} bytes)")
    
    if received_messages:
        print("✅ Fleet API -> Zenoh communication working")
        print("💡 Now checking if robot received messages...")
        
        # Check robot logs
        import subprocess
        try:
            result = subprocess.run(
                ["docker", "logs", "teleop-robot-robot-1", "--tail", "10"],
                capture_output=True, text=True, timeout=10
            )
            print("\n🤖 Robot logs:")
            print(result.stdout)
            
            if "velocity_x=" in result.stdout:
                print("✅ Robot received messages!")
            else:
                print("❌ Robot did not receive messages")
                print("💡 Bridge is not forwarding Zenoh -> ROS2")
                
        except Exception as e:
            print(f"Error checking robot logs: {e}")
    else:
        print("❌ Fleet API -> Zenoh communication not working")

if __name__ == "__main__":
    asyncio.run(monitor_zenoh_and_test_fleet())