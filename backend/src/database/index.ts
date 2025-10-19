export { DatabaseConnection, initializeDatabase, gracefulShutdown } from './connection';
export { VelocityLogModel, DatabaseService, createDatabaseService } from './models';
export { 
  calculateVelocityFromLevels, 
  validateSpeedLevels, 
  validateVelocity,
  sanitizeLogData,
  createDatabaseError
} from './utils';

export type { VelocityLogEntry, SpeedLevels, ValidationResult } from '@web-teleop-robot/shared';