/**
 * Database module exports
 * Provides MongoDB connection utilities, models, and database operations
 */

export { DatabaseConnection, initializeDatabase, gracefulShutdown } from './connection';
export { VelocityLogModel, DatabaseService, createDatabaseService } from './models';
export { 
  calculateVelocityFromLevels, 
  validateSpeedLevels, 
  validateVelocity,
  sanitizeLogData,
  createDatabaseError
} from './utils';

// Re-export shared types for convenience
export type { VelocityLogEntry, SpeedLevels, ValidationResult } from '@web-teleop-robot/shared';