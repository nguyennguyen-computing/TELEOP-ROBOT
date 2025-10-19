/**
 * Jest test setup configuration
 */

// Increase timeout for database operations
jest.setTimeout(30000);

// Mock environment variables for testing
process.env.NODE_ENV = 'test';
process.env.NODE_PORT = '3001';
process.env.FLEET_API_URL = 'http://localhost:8000';
process.env.MONGO_URI = 'mongodb://localhost:27017/test_teleop_db';
process.env.MONGO_DB = 'test_teleop_db';
process.env.CORS_ORIGIN = 'http://localhost:3000';
process.env.VX_MAX = '1.0';
process.env.VY_MAX = '1.0';
process.env.ZENOH_LOCATOR = 'tcp/localhost:7447';
process.env.Z_KEY_CMD_VEL = 'rt/ros2/cmd_vel';

// Suppress console logs during testing unless explicitly needed
if (process.env.VERBOSE_TESTS !== 'true') {
  console.log = jest.fn();
  console.warn = jest.fn();
  console.error = jest.fn();
}