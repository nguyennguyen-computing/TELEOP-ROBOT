import type { EnvironmentConfig } from '@web-teleop-robot/shared';

/**
 * Validates and parses environment variables
 * @returns Validated environment configuration
 * @throws Error if required environment variables are missing or invalid
 */
export function validateEnvironment(): EnvironmentConfig {
  const requiredEnvVars = [
    'NODE_PORT',
    'FLEET_API_URL',
    'MONGO_URI',
    'CORS_ORIGIN',
    'VX_MAX',
    'VY_MAX',
    'ZENOH_LOCATOR',
    'Z_KEY_CMD_VEL',
  ];

  const authEnabled = process.env.AUTH_ENABLED === 'true';
  if (authEnabled && !process.env.JWT_SECRET) {
    throw new Error('JWT_SECRET is required when AUTH_ENABLED is true');
  }

  const missingVars = requiredEnvVars.filter(varName => !process.env[varName]);
  if (missingVars.length > 0) {
    throw new Error(`Missing required environment variables: ${missingVars.join(', ')}`);
  }

  const nodePort = parseInt(process.env.NODE_PORT!, 10);
  const webPort = parseInt(process.env.WEB_PORT || '3000', 10);
  const fleetPort = parseInt(process.env.FLEET_PORT || '8000', 10);
  const mongoPort = parseInt(process.env.MONGO_PORT || '27017', 10);
  const zenohPort = parseInt(process.env.ZENOH_PORT || '7447', 10);
  const vxMax = parseFloat(process.env.VX_MAX!);
  const vyMax = parseFloat(process.env.VY_MAX!);
  const rosDomainId = parseInt(process.env.ROS_DOMAIN_ID || '0', 10);

  // Validate numeric values
  if (isNaN(nodePort) || nodePort <= 0 || nodePort > 65535) {
    throw new Error('NODE_PORT must be a valid port number (1-65535)');
  }

  if (isNaN(vxMax) || vxMax <= 0) {
    throw new Error('VX_MAX must be a positive number');
  }

  if (isNaN(vyMax) || vyMax <= 0) {
    throw new Error('VY_MAX must be a positive number');
  }

  if (isNaN(rosDomainId) || rosDomainId < 0 || rosDomainId > 232) {
    throw new Error('ROS_DOMAIN_ID must be between 0 and 232');
  }

  // Validate URLs
  try {
    new URL(process.env.FLEET_API_URL!);
  } catch {
    throw new Error('FLEET_API_URL must be a valid URL');
  }

  try {
    new URL(process.env.CORS_ORIGIN!);
  } catch {
    throw new Error('CORS_ORIGIN must be a valid URL');
  }

  const authEnabledConfig = process.env.AUTH_ENABLED === 'true';
  const apiKeysString = process.env.API_KEYS || '';
  const jwtSecret = process.env.JWT_SECRET || '';
  const jwtExpiresIn = process.env.JWT_EXPIRES_IN || '24h';
  const rateLimitWindowMs = parseInt(process.env.RATE_LIMIT_WINDOW_MS || '900000', 10); // 15 minutes
  const rateLimitMaxRequests = parseInt(process.env.RATE_LIMIT_MAX_REQUESTS || '100', 10);

  const config: EnvironmentConfig = {
    WEB_PORT: webPort,
    NODE_PORT: nodePort,
    FLEET_PORT: fleetPort,
    MONGO_PORT: mongoPort,
    ZENOH_PORT: zenohPort,
    NODE_API_URL: process.env.NODE_API_URL || `http://localhost:${nodePort}`,
    NODE_WS_URL: process.env.NODE_WS_URL || `ws://localhost:${nodePort}`,
    FLEET_API_URL: process.env.FLEET_API_URL!,
    MONGO_URI: process.env.MONGO_URI!,
    CORS_ORIGIN: process.env.CORS_ORIGIN!,
    VX_MAX: vxMax,
    VY_MAX: vyMax,
    ZENOH_LOCATOR: process.env.ZENOH_LOCATOR!,
    Z_KEY_CMD_VEL: process.env.Z_KEY_CMD_VEL!,
    ROS_DOMAIN_ID: rosDomainId,
    // Authentication configuration
    AUTH_ENABLED: authEnabledConfig,
    API_KEYS: apiKeysString,
    JWT_SECRET: jwtSecret,
    JWT_EXPIRES_IN: jwtExpiresIn,
    RATE_LIMIT_WINDOW_MS: rateLimitWindowMs,
    RATE_LIMIT_MAX_REQUESTS: rateLimitMaxRequests,
  };

  return config;
}

/**
 * Gets the current environment configuration
 * @returns Environment configuration object
 */
export function getEnvironmentConfig(): EnvironmentConfig {
  return validateEnvironment();
}

/**
 * Checks if the application is running in development mode
 * @returns True if in development mode
 */
export function isDevelopment(): boolean {
  return process.env.NODE_ENV === 'development';
}

/**
 * Checks if the application is running in production mode
 * @returns True if in production mode
 */
export function isProduction(): boolean {
  return process.env.NODE_ENV === 'production';
}

/**
 * Checks if the application is running in test mode
 * @returns True if in test mode
 */
export function isTest(): boolean {
  return process.env.NODE_ENV === 'test';
}