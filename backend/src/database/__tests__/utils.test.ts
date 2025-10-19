/**
 * Unit tests for database utility functions
 */

import { 
  calculateVelocityFromLevels, 
  validateSpeedLevels, 
  validateVelocity,
  sanitizeLogData
} from '../utils';
import { SpeedLevels } from '@web-teleop-robot/shared';

describe('Database Utils', () => {
  describe('calculateVelocityFromLevels', () => {
    it('should calculate correct velocity for forward movement', () => {
      const levels: SpeedLevels = { up: 5, down: 0, left: 0, right: 0 };
      const result = calculateVelocityFromLevels(levels, 1.0, 1.0);
      
      expect(result.vx).toBe(0.5); // 5 * (1.0/10)
      expect(result.vy).toBe(0.0);
    });

    it('should calculate correct velocity for backward movement', () => {
      const levels: SpeedLevels = { up: 0, down: 3, left: 0, right: 0 };
      const result = calculateVelocityFromLevels(levels, 1.0, 1.0);
      
      expect(result.vx).toBeCloseTo(-0.3); // -3 * (1.0/10)
      expect(result.vy).toBe(0.0);
    });

    it('should calculate correct velocity for right movement', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 0, right: 7 };
      const result = calculateVelocityFromLevels(levels, 1.0, 1.0);
      
      expect(result.vx).toBe(0.0);
      expect(result.vy).toBeCloseTo(0.7); // 7 * (1.0/10)
    });

    it('should calculate correct velocity for left movement', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 4, right: 0 };
      const result = calculateVelocityFromLevels(levels, 1.0, 1.0);
      
      expect(result.vx).toBe(0.0);
      expect(result.vy).toBe(-0.4); // -4 * (1.0/10)
    });

    it('should calculate correct velocity for diagonal movement', () => {
      const levels: SpeedLevels = { up: 6, down: 0, left: 0, right: 8 };
      const result = calculateVelocityFromLevels(levels, 1.0, 1.0);
      
      expect(result.vx).toBeCloseTo(0.6); // 6 * (1.0/10)
      expect(result.vy).toBeCloseTo(0.8); // 8 * (1.0/10)
    });

    it('should handle opposing directions correctly', () => {
      const levels: SpeedLevels = { up: 8, down: 3, left: 2, right: 7 };
      const result = calculateVelocityFromLevels(levels, 1.0, 1.0);
      
      expect(result.vx).toBe(0.5); // (8-3) * (1.0/10)
      expect(result.vy).toBe(0.5); // (7-2) * (1.0/10)
    });

    it('should clamp values to maximum range', () => {
      const levels: SpeedLevels = { up: 15, down: 0, left: 0, right: 12 };
      const result = calculateVelocityFromLevels(levels, 1.0, 1.0);
      
      expect(result.vx).toBe(1.0); // Clamped to max
      expect(result.vy).toBe(1.0); // Clamped to max
    });

    it('should work with custom velocity limits', () => {
      const levels: SpeedLevels = { up: 5, down: 0, left: 0, right: 0 };
      const result = calculateVelocityFromLevels(levels, 2.0, 1.5);
      
      expect(result.vx).toBe(1.0); // 5 * (2.0/10)
      expect(result.vy).toBe(0.0);
    });
  });

  describe('validateSpeedLevels', () => {
    it('should validate correct speed levels', () => {
      const levels: SpeedLevels = { up: 5, down: 3, left: 0, right: 7 };
      const result = validateSpeedLevels(levels);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should reject null or undefined levels', () => {
      const result = validateSpeedLevels(null as any);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Levels must be an object');
    });

    it('should reject missing properties', () => {
      const levels = { up: 5, down: 3 } as any;
      const result = validateSpeedLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('left must be a number');
      expect(result.errors).toContain('right must be a number');
    });

    it('should reject non-numeric values', () => {
      const levels = { up: 'five', down: 3, left: 0, right: 7 } as any;
      const result = validateSpeedLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('up must be a number');
    });

    it('should reject values outside valid range', () => {
      const levels: SpeedLevels = { up: 15, down: -5, left: 0, right: 7 };
      const result = validateSpeedLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('up must be between 0 and 10');
      expect(result.errors).toContain('down must be between 0 and 10');
    });

    it('should reject non-integer values', () => {
      const levels: SpeedLevels = { up: 5.5, down: 3, left: 0, right: 7 };
      const result = validateSpeedLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('up must be an integer');
    });

    it('should reject NaN values', () => {
      const levels: SpeedLevels = { up: NaN, down: 3, left: 0, right: 7 };
      const result = validateSpeedLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('up cannot be NaN');
    });
  });

  describe('validateVelocity', () => {
    it('should validate correct velocity values', () => {
      const result = validateVelocity(0.5, -0.3, 1.0, 1.0);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    it('should reject velocity values exceeding limits', () => {
      const result = validateVelocity(1.5, -1.2, 1.0, 1.0);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('vx must be between -1 and 1');
      expect(result.errors).toContain('vy must be between -1 and 1');
    });

    it('should reject non-numeric velocity values', () => {
      const result = validateVelocity(NaN, 'invalid' as any, 1.0, 1.0);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('vx must be a valid number');
      expect(result.errors).toContain('vy must be a valid number');
    });

    it('should work with custom velocity limits', () => {
      const result = validateVelocity(1.5, -1.8, 2.0, 2.0);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });
  });

  describe('sanitizeLogData', () => {
    it('should sanitize valid data correctly', () => {
      const input = {
        vx: 0.5,
        vy: -0.3,
        levels: { up: 5, down: 0, left: 3, right: 0 },
        source: 'web'
      };
      
      const result = sanitizeLogData(input);
      
      expect(result).toEqual(input);
    });

    it('should clamp level values to valid range', () => {
      const input = {
        vx: 0.5,
        vy: -0.3,
        levels: { up: 15, down: -5, left: 3.7, right: 0 },
        source: 'web'
      };
      
      const result = sanitizeLogData(input);
      
      expect(result.levels.up).toBe(10);    // Clamped from 15
      expect(result.levels.down).toBe(0);   // Clamped from -5
      expect(result.levels.left).toBe(3);   // Floored from 3.7
      expect(result.levels.right).toBe(0);
    });

    it('should provide default source when missing', () => {
      const input = {
        vx: 0.5,
        vy: -0.3,
        levels: { up: 5, down: 0, left: 0, right: 0 }
      };
      
      const result = sanitizeLogData(input);
      
      expect(result.source).toBe('web');
    });

    it('should handle invalid numeric values', () => {
      const input = {
        vx: NaN,
        vy: 'invalid' as any,
        levels: { up: null as any, down: undefined as any, left: 0, right: 0 },
        source: 'web'
      };
      
      const result = sanitizeLogData(input);
      
      expect(result.vx).toBe(0);
      expect(result.vy).toBe(0);
      expect(result.levels.up).toBe(0);
      expect(result.levels.down).toBe(0);
    });
  });
});