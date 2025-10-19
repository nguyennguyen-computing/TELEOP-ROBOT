/**
 * Database utility functions for common operations
 */

import { SpeedLevels } from '@web-teleop-robot/shared';

export function calculateVelocityFromLevels(
  levels: SpeedLevels,
  vxMax: number = 1.0,
  vyMax: number = 1.0
): { vx: number; vy: number } {
  // Calculate net levels
  const netXLevel = levels.up - levels.down;    // Forward = +X, Backward = -X
  const netYLevel = levels.right - levels.left; // Right = +Y, Left = -Y
  
  // Clamp to valid range [-10, 10]
  const clampedXLevel = Math.max(-10, Math.min(10, netXLevel));
  const clampedYLevel = Math.max(-10, Math.min(10, netYLevel));
  
  // Calculate step sizes
  const stepX = vxMax / 10;
  const stepY = vyMax / 10;
  
  // Calculate final velocities
  const vx = clampedXLevel * stepX;
  const vy = clampedYLevel * stepY;
  
  return { vx, vy };
}

/**
 * Validate speed levels
 */
export function validateSpeedLevels(levels: SpeedLevels): { isValid: boolean; errors: string[] } {
  const errors: string[] = [];
  
  if (!levels || typeof levels !== 'object') {
    errors.push('Levels must be an object');
    return { isValid: false, errors };
  }
  
  const requiredKeys = ['up', 'down', 'left', 'right'];
  
  for (const key of requiredKeys) {
    const value = levels[key as keyof SpeedLevels];
    
    if (typeof value !== 'number') {
      errors.push(`${key} must be a number`);
    } else if (isNaN(value)) {
      errors.push(`${key} cannot be NaN`);
    } else if (value < 0 || value > 10) {
      errors.push(`${key} must be between 0 and 10`);
    } else if (!Number.isInteger(value)) {
      errors.push(`${key} must be an integer`);
    }
  }
  
  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Validate velocity values
 */
export function validateVelocity(
  vx: number, 
  vy: number, 
  vxMax: number = 1.0, 
  vyMax: number = 1.0
): { isValid: boolean; errors: string[] } {
  const errors: string[] = [];
  
  if (typeof vx !== 'number' || isNaN(vx)) {
    errors.push('vx must be a valid number');
  } else if (Math.abs(vx) > vxMax) {
    errors.push(`vx must be between -${vxMax} and ${vxMax}`);
  }
  
  if (typeof vy !== 'number' || isNaN(vy)) {
    errors.push('vy must be a valid number');
  } else if (Math.abs(vy) > vyMax) {
    errors.push(`vy must be between -${vyMax} and ${vyMax}`);
  }
  
  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Create a standardized database error response
 */
export function createDatabaseError(operation: string, error: Error): Error {
  const message = `Database ${operation} failed: ${error.message}`;
  const dbError = new Error(message);
  dbError.name = 'DatabaseError';
  return dbError;
}

/**
 * Sanitize log data for safe database insertion
 */
export function sanitizeLogData(data: {
  vx: number;
  vy: number;
  levels: SpeedLevels;
  source?: string;
}): {
  vx: number;
  vy: number;
  levels: SpeedLevels;
  source: string;
} {
  return {
    vx: Number(data.vx) || 0,
    vy: Number(data.vy) || 0,
    levels: {
      up: Math.max(0, Math.min(10, Math.floor(Number(data.levels.up) || 0))),
      down: Math.max(0, Math.min(10, Math.floor(Number(data.levels.down) || 0))),
      left: Math.max(0, Math.min(10, Math.floor(Number(data.levels.left) || 0))),
      right: Math.max(0, Math.min(10, Math.floor(Number(data.levels.right) || 0)))
    },
    source: data.source || 'web'
  };
}