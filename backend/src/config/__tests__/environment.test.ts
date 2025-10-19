/**
 * Tests for environment configuration validation
 */

import { validateEnvironment, isDevelopment, isProduction, isTest } from '../environment';

describe('Environment Configuration', () => {
  const originalEnv = process.env;

  beforeEach(() => {
    // Reset environment variables
    jest.resetModules();
    process.env = { ...originalEnv };
  });

  afterAll(() => {
    process.env = originalEnv;
  });

  describe('validateEnvironment', () => {
    const requiredEnvVars = {
      NODE_PORT: '3001',
      FLEET_API_URL: 'http://localhost:8000',
      MONGO_URI: 'mongodb://localhost:27017/test',
      CORS_ORIGIN: 'http://localhost:3000',
      VX_MAX: '1.0',
      VY_MAX: '1.0',
      ZENOH_LOCATOR: 'tcp/localhost:7447',
      Z_KEY_CMD_VEL: 'rt/ros2/cmd_vel',
    };

    beforeEach(() => {
      // Set all required environment variables
      Object.assign(process.env, requiredEnvVars);
    });

    it('should validate and return configuration with all required variables', () => {
      const config = validateEnvironment();

      expect(config).toMatchObject({
        NODE_PORT: 3001,
        FLEET_API_URL: 'http://localhost:8000',
        MONGO_URI: 'mongodb://localhost:27017/test',
        CORS_ORIGIN: 'http://localhost:3000',
        VX_MAX: 1.0,
        VY_MAX: 1.0,
        ZENOH_LOCATOR: 'tcp/localhost:7447',
        Z_KEY_CMD_VEL: 'rt/ros2/cmd_vel',
      });
    });

    it('should throw error for missing required variables', () => {
      delete process.env.NODE_PORT;
      delete process.env.VX_MAX;

      expect(() => validateEnvironment()).toThrow(
        'Missing required environment variables: NODE_PORT, VX_MAX'
      );
    });

    it('should throw error for invalid port number', () => {
      process.env.NODE_PORT = 'invalid';

      expect(() => validateEnvironment()).toThrow(
        'NODE_PORT must be a valid port number'
      );
    });

    it('should throw error for port out of range', () => {
      process.env.NODE_PORT = '70000';

      expect(() => validateEnvironment()).toThrow(
        'NODE_PORT must be a valid port number'
      );
    });

    it('should throw error for invalid velocity values', () => {
      process.env.VX_MAX = '-1.0';

      expect(() => validateEnvironment()).toThrow(
        'VX_MAX must be a positive number'
      );
    });

    it('should throw error for invalid URLs', () => {
      process.env.FLEET_API_URL = 'not-a-url';

      expect(() => validateEnvironment()).toThrow(
        'FLEET_API_URL must be a valid URL'
      );
    });

    it('should use default values for optional variables', () => {
      delete process.env.WEB_PORT;
      delete process.env.ROS_DOMAIN_ID;

      const config = validateEnvironment();

      expect(config.WEB_PORT).toBe(3000);
      expect(config.ROS_DOMAIN_ID).toBe(0);
    });

    it('should validate ROS domain ID range', () => {
      process.env.ROS_DOMAIN_ID = '300';

      expect(() => validateEnvironment()).toThrow(
        'ROS_DOMAIN_ID must be between 0 and 232'
      );
    });
  });

  describe('Environment mode helpers', () => {
    it('should detect development mode', () => {
      process.env.NODE_ENV = 'development';
      expect(isDevelopment()).toBe(true);
      expect(isProduction()).toBe(false);
      expect(isTest()).toBe(false);
    });

    it('should detect production mode', () => {
      process.env.NODE_ENV = 'production';
      expect(isDevelopment()).toBe(false);
      expect(isProduction()).toBe(true);
      expect(isTest()).toBe(false);
    });

    it('should detect test mode', () => {
      process.env.NODE_ENV = 'test';
      expect(isDevelopment()).toBe(false);
      expect(isProduction()).toBe(false);
      expect(isTest()).toBe(true);
    });
  });
});