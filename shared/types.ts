/**
 * Shared TypeScript interfaces and types for the Web Teleop Robot system
 */

import { Request } from 'express';

// Core velocity command interface
export interface VelocityCommand {
  vx: number;           // Velocity X (-VX_MAX to +VX_MAX)
  vy: number;           // Velocity Y (-VY_MAX to +VY_MAX)
  levels: SpeedLevels;  // Speed levels for each direction
  timestamp?: Date;     // Command timestamp
  source?: string;      // Command source identifier
}

// Speed levels for directional control
export interface SpeedLevels {
  up: number;           // Forward level (0-10)
  down: number;         // Backward level (0-10)
  left: number;         // Left level (0-10)
  right: number;        // Right level (0-10)
}

// System configuration interface
export interface SystemConfig {
  // Velocity limits
  VX_MAX: number;       // Maximum X velocity (m/s)
  VY_MAX: number;       // Maximum Y velocity (m/s)
  
  // Computed step sizes
  STEP_X: number;       // VX_MAX / 10
  STEP_Y: number;       // VY_MAX / 10
  
  // Network configuration
  NODE_API_URL: string;
  FLEET_API_URL: string;
  ZENOH_LOCATOR: string;
  
  // Zenoh keys
  Z_KEY_CMD_VEL: string;
  
  // Database
  MONGO_URI: string;
  MONGO_DB: string;
}

// API response interfaces
export interface ApiResponse<T = any> {
  ok: boolean;
  data?: T;
  error?: string;
  timestamp: string;
}

export interface VelocityResponse extends ApiResponse {
  published?: boolean;
}

// Database log entry interface
export interface VelocityLogEntry {
  _id?: string;
  ts: Date;             // Timestamp
  vx: number;           // Velocity X
  vy: number;           // Velocity Y
  levels: SpeedLevels;  // Speed levels
  source: string;       // Command source ('web')
}

// WebSocket message types
export interface WebSocketMessage {
  type: 'set_vel' | 'connection_status' | 'error' | 'velocity_update' | 'ping' | 'pong';
  data: any;
  timestamp: string;
}

export interface SetVelMessage extends WebSocketMessage {
  type: 'set_vel';
  data: VelocityCommand;
}

export interface ConnectionStatusMessage extends WebSocketMessage {
  type: 'connection_status';
  data: {
    connected: boolean;
    clientCount: number;
    connectionId?: string;
    connectedAt?: string;
    lastPing?: string;
    message?: string;
  };
}

export interface VelocityUpdateMessage extends WebSocketMessage {
  type: 'velocity_update';
  data: {
    vx: number;
    vy: number;
    levels: SpeedLevels;
    source: string;
    timestamp?: string;
  };
}

export interface ErrorMessage extends WebSocketMessage {
  type: 'error';
  data: {
    message: string;
    errors?: string[];
  };
}

// ROS2 Twist message interface
export interface TwistMessage {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
}

// Frontend state interfaces
export interface RobotState {
  levels: SpeedLevels;
  velocity: { vx: number; vy: number };
  connectionStatus: 'connected' | 'disconnected' | 'connecting';
  isHolding: { [key: string]: boolean };
  
  // UI/UX specific state
  theme: 'light' | 'dark';
  notifications: Notification[];
  isFullscreen: boolean;
  hapticFeedback: boolean;
  soundEnabled: boolean;
  lastCommandTime: Date | null;
  commandHistory: VelocityCommand[];
}

export interface Notification {
  id: string;
  type: 'success' | 'warning' | 'error' | 'info';
  message: string;
  duration?: number;
  timestamp: Date;
}

// Direction type for UI components
export type Direction = 'up' | 'down' | 'left' | 'right';

// Button state for UI feedback
export interface ButtonState {
  isPressed: boolean;
  isHolding: boolean;
  level: number;
}

// Validation result interface
export interface ValidationResult {
  isValid: boolean;
  errors: string[];
}

// Constants
export const VELOCITY_CONSTANTS = {
  MAX_LEVEL: 10,
  MIN_LEVEL: 0,
  HOLD_REPEAT_RATE_HZ: 15, // 10-20 Hz as specified
  HOLD_REPEAT_INTERVAL: 1000 / 15, // milliseconds
} as const;

// Authentication interfaces
export interface AuthConfig {
  enabled: boolean;
  apiKeys: string[];
  jwt: {
    secret: string;
    expiresIn: string;
    algorithm: 'HS256' | 'HS384' | 'HS512';
  };
  rateLimit: {
    windowMs: number;
    maxRequests: number;
  };
}

export interface JWTPayload {
  sub: string;          // Subject (user ID)
  iat: number;          // Issued at
  exp: number;          // Expiration time
  scope: string[];      // Permissions/scopes
}

export interface AuthRequest {
  apiKey?: string;      // API key from header or query
  token?: string;       // JWT token from Authorization header
  user?: {              // Authenticated user info
    id: string;
    scopes: string[];
  };
}

export interface AuthenticatedRequest extends Request {
  auth?: AuthRequest;
}

// Utility type for environment variables
export interface EnvironmentConfig {
  WEB_PORT: number;
  NODE_PORT: number;
  FLEET_PORT: number;
  MONGO_PORT: number;
  ZENOH_PORT: number;
  NODE_API_URL: string;
  NODE_WS_URL: string;
  FLEET_API_URL: string;
  MONGO_URI: string;
  CORS_ORIGIN: string;
  VX_MAX: number;
  VY_MAX: number;
  ZENOH_LOCATOR: string;
  Z_KEY_CMD_VEL: string;
  ROS_DOMAIN_ID: number;
  // Authentication configuration
  AUTH_ENABLED?: boolean;
  API_KEYS?: string;
  JWT_SECRET?: string;
  JWT_EXPIRES_IN?: string;
  RATE_LIMIT_WINDOW_MS?: number;
  RATE_LIMIT_MAX_REQUESTS?: number;
}