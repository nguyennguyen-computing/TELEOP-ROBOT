/**
 * Shared module exports for Web Teleop Robot system
 */

// Export all types
export * from './types';

// Export all utilities
export * from './utils';

// Re-export commonly used items for convenience
export {
  VelocityCommand,
  SpeedLevels,
  SystemConfig,
  RobotState,
  Direction,
  VELOCITY_CONSTANTS
} from './types';

export {
  calculateVelocity,
  validateVelocityCommand,
  validateSpeedLevels,
  createTwistMessage,
  clamp,
  generateId,
  formatTimestamp,
  parseEnvNumber,
  parseEnvInt,
  createInitialLevels,
  isStoppedState,
  incrementLevel,
  resetLevels
} from './utils';