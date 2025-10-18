/**
 * Shared utility functions for the Web Teleop Robot system
 */

import { VelocityCommand, SpeedLevels, ValidationResult, VELOCITY_CONSTANTS } from './types';

/**
 * Calculate velocity from speed levels using the coordinate system:
 * Forward = +X, Backward = -X, Right = +Y, Left = -Y
 */
export function calculateVelocity(
  levels: SpeedLevels,
  stepX: number,
  stepY: number
): { vx: number; vy: number } {
  // Calculate net levels
  const netXLevel = levels.up - levels.down;    // Forward - Backward
  const netYLevel = levels.right - levels.left; // Right - Left
  
  // Clamp to valid range and scale
  const vx = clamp(netXLevel, -VELOCITY_CONSTANTS.MAX_LEVEL, VELOCITY_CONSTANTS.MAX_LEVEL) * stepX;
  const vy = clamp(netYLevel, -VELOCITY_CONSTANTS.MAX_LEVEL, VELOCITY_CONSTANTS.MAX_LEVEL) * stepY;
  
  return { vx, vy };
}

/**
 * Clamp a value between min and max
 */
export function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}

/**
 * Validate velocity command data
 */
export function validateVelocityCommand(
  data: Partial<VelocityCommand>,
  vxMax: number,
  vyMax: number
): ValidationResult {
  const errors: string[] = [];
  
  // Check required fields
  if (typeof data.vx !== 'number') {
    errors.push('vx must be a number');
  } else if (Math.abs(data.vx) > vxMax) {
    errors.push(`vx must be between -${vxMax} and ${vxMax}`);
  }
  
  if (typeof data.vy !== 'number') {
    errors.push('vy must be a number');
  } else if (Math.abs(data.vy) > vyMax) {
    errors.push(`vy must be between -${vyMax} and ${vyMax}`);
  }
  
  // Validate levels
  if (!data.levels || typeof data.levels !== 'object') {
    errors.push('levels must be an object');
  } else {
    const { up, down, left, right } = data.levels;
    
    for (const [direction, level] of Object.entries({ up, down, left, right })) {
      if (typeof level !== 'number') {
        errors.push(`levels.${direction} must be a number`);
      } else if (level < VELOCITY_CONSTANTS.MIN_LEVEL || level > VELOCITY_CONSTANTS.MAX_LEVEL) {
        errors.push(`levels.${direction} must be between ${VELOCITY_CONSTANTS.MIN_LEVEL} and ${VELOCITY_CONSTANTS.MAX_LEVEL}`);
      }
    }
  }
  
  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Validate speed levels object
 */
export function validateSpeedLevels(levels: Partial<SpeedLevels>): ValidationResult {
  const errors: string[] = [];
  const requiredFields = ['up', 'down', 'left', 'right'] as const;
  
  for (const field of requiredFields) {
    const value = levels[field];
    if (typeof value !== 'number') {
      errors.push(`${field} must be a number`);
    } else if (value < VELOCITY_CONSTANTS.MIN_LEVEL || value > VELOCITY_CONSTANTS.MAX_LEVEL) {
      errors.push(`${field} must be between ${VELOCITY_CONSTANTS.MIN_LEVEL} and ${VELOCITY_CONSTANTS.MAX_LEVEL}`);
    }
  }
  
  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Create a ROS2 Twist message from velocity values
 */
export function createTwistMessage(vx: number, vy: number) {
  return {
    linear: {
      x: vx,
      y: vy,
      z: 0.0
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: 0.0
    }
  };
}

/**
 * Generate a unique ID for notifications and other purposes
 */
export function generateId(): string {
  return `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
}

/**
 * Format timestamp for logging
 */
export function formatTimestamp(date: Date = new Date()): string {
  return date.toISOString();
}

/**
 * Parse environment variable as number with default
 */
export function parseEnvNumber(value: string | undefined, defaultValue: number): number {
  if (!value) return defaultValue;
  const parsed = parseFloat(value);
  return isNaN(parsed) ? defaultValue : parsed;
}

/**
 * Parse environment variable as integer with default
 */
export function parseEnvInt(value: string | undefined, defaultValue: number): number {
  if (!value) return defaultValue;
  const parsed = parseInt(value, 10);
  return isNaN(parsed) ? defaultValue : parsed;
}

/**
 * Create initial speed levels (all zeros)
 */
export function createInitialLevels(): SpeedLevels {
  return {
    up: 0,
    down: 0,
    left: 0,
    right: 0
  };
}

/**
 * Check if all speed levels are zero (stopped state)
 */
export function isStoppedState(levels: SpeedLevels): boolean {
  return levels.up === 0 && levels.down === 0 && levels.left === 0 && levels.right === 0;
}

/**
 * Increment a speed level by 1, clamped to max
 */
export function incrementLevel(currentLevel: number): number {
  return Math.min(currentLevel + 1, VELOCITY_CONSTANTS.MAX_LEVEL);
}

/**
 * Reset all levels to zero
 */
export function resetLevels(): SpeedLevels {
  return createInitialLevels();
}