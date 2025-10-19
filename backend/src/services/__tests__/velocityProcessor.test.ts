/**
 * Unit tests for VelocityProcessor
 * 
 * Tests all velocity calculation logic, validation, and coordinate system mapping
 * according to requirements 5.1-5.7
 */

import { VelocityProcessor, VelocityUtils } from '../velocityProcessor';
import type { SpeedLevels, VelocityCommand } from '@web-teleop-robot/shared';

// Mock environment configuration
jest.mock('../../config/environment', () => ({
  getEnvironmentConfig: () => ({
    VX_MAX: 1.0,
    VY_MAX: 0.8,
    NODE_PORT: 3001,
    FLEET_API_URL: 'http://localhost:8000',
    MONGO_URI: 'mongodb://localhost:27017/test',
    CORS_ORIGIN: 'http://localhost:3000',
    ZENOH_LOCATOR: 'tcp/localhost:7447',
    Z_KEY_CMD_VEL: 'rt/ros2/cmd_vel',
    ROS_DOMAIN_ID: 0
  })
}));

describe('VelocityProcessor', () => {
  let processor: VelocityProcessor;

  beforeEach(() => {
    processor = new VelocityProcessor();
  });

  describe('Coordinate System Mapping', () => {
    // Requirement 5.1: Forward = +X, Backward = -X
    test('should map forward movement to positive X velocity', () => {
      const levels: SpeedLevels = { up: 5, down: 0, left: 0, right: 0 };
      const { vx, vy } = processor.calculateVelocity(levels);
      
      expect(vx).toBeGreaterThan(0);
      expect(vy).toBe(0);
      expect(vx).toBe(5 * 0.1); // 5 levels * (1.0 / 10)
    });

    test('should map backward movement to negative X velocity', () => {
      const levels: SpeedLevels = { up: 0, down: 3, left: 0, right: 0 };
      const { vx, vy } = processor.calculateVelocity(levels);
      
      expect(vx).toBeLessThan(0);
      expect(vy).toBe(0);
      expect(vx).toBe(-3 * 0.1); // -3 levels * (1.0 / 10)
    });

    // Requirement 5.2: Right = +Y, Left = -Y
    test('should map right movement to positive Y velocity', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 0, right: 7 };
      const { vx, vy } = processor.calculateVelocity(levels);
      
      expect(vx).toBe(0);
      expect(vy).toBeGreaterThan(0);
      expect(vy).toBe(7 * 0.08); // 7 levels * (0.8 / 10)
    });

    test('should map left movement to negative Y velocity', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 4, right: 0 };
      const { vx, vy } = processor.calculateVelocity(levels);
      
      expect(vx).toBe(0);
      expect(vy).toBeLessThan(0);
      expect(vy).toBe(-4 * 0.08); // -4 levels * (0.8 / 10)
    });
  });

  describe('Net Level Calculations', () => {
    // Requirement 5.3: net_x_level = up - down
    test('should calculate net X level correctly', () => {
      const levels: SpeedLevels = { up: 8, down: 3, left: 0, right: 0 };
      const { vx } = processor.calculateVelocity(levels);
      
      const expectedNetX = 8 - 3; // = 5
      const expectedVx = expectedNetX * 0.1; // 5 * (1.0 / 10)
      expect(vx).toBe(expectedVx);
    });

    // Requirement 5.4: net_y_level = right - left
    test('should calculate net Y level correctly', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 2, right: 9 };
      const { vy } = processor.calculateVelocity(levels);
      
      const expectedNetY = 9 - 2; // = 7
      const expectedVy = expectedNetY * 0.08; // 7 * (0.8 / 10)
      expect(vy).toBe(expectedVy);
    });

    test('should handle opposing directions correctly', () => {
      const levels: SpeedLevels = { up: 6, down: 6, left: 4, right: 4 };
      const { vx, vy } = processor.calculateVelocity(levels);
      
      expect(vx).toBe(0); // 6 - 6 = 0
      expect(vy).toBe(0); // 4 - 4 = 0
    });
  });

  describe('Velocity Clamping and Scaling', () => {
    // Requirement 5.5: vel_x = clamp(net_x_level, -10, 10) × STEP_X
    test('should clamp X velocity to maximum range', () => {
      // Test levels that would exceed maximum
      const levels: SpeedLevels = { up: 10, down: 0, left: 0, right: 0 };
      const { vx } = processor.calculateVelocity(levels);
      
      expect(vx).toBe(1.0); // 10 * 0.1 = 1.0 (VX_MAX)
    });

    test('should clamp X velocity to minimum range', () => {
      const levels: SpeedLevels = { up: 0, down: 10, left: 0, right: 0 };
      const { vx } = processor.calculateVelocity(levels);
      
      expect(vx).toBe(-1.0); // -10 * 0.1 = -1.0 (-VX_MAX)
    });

    // Requirement 5.6: vel_y = clamp(net_y_level, -10, 10) × STEP_Y
    test('should clamp Y velocity to maximum range', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 0, right: 10 };
      const { vy } = processor.calculateVelocity(levels);
      
      expect(vy).toBe(0.8); // 10 * 0.08 = 0.8 (VY_MAX)
    });

    test('should clamp Y velocity to minimum range', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 10, right: 0 };
      const { vy } = processor.calculateVelocity(levels);
      
      expect(vy).toBe(-0.8); // -10 * 0.08 = -0.8 (-VY_MAX)
    });

    test('should handle intermediate scaling correctly', () => {
      const levels: SpeedLevels = { up: 5, down: 2, left: 1, right: 6 };
      const { vx, vy } = processor.calculateVelocity(levels);
      
      // Net levels: up - down = 5 - 2 = 3, right - left = 6 - 1 = 5
      expect(vx).toBe(3 * 0.1); // 0.3
      expect(vy).toBe(5 * 0.08); // 0.4
    });
  });

  describe('Level Validation', () => {
    test('should validate correct levels', () => {
      const levels: SpeedLevels = { up: 5, down: 3, left: 0, right: 10 };
      const result = processor.validateLevels(levels);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    test('should reject negative levels', () => {
      const levels: SpeedLevels = { up: -1, down: 3, left: 0, right: 5 };
      const result = processor.validateLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain("Level 'up' must be between 0 and 10, got -1");
    });

    test('should reject levels above 10', () => {
      const levels: SpeedLevels = { up: 5, down: 11, left: 0, right: 5 };
      const result = processor.validateLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain("Level 'down' must be between 0 and 10, got 11");
    });

    test('should reject non-integer levels', () => {
      const levels: SpeedLevels = { up: 5.5, down: 3, left: 0, right: 5 };
      const result = processor.validateLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain("Level 'up' must be an integer");
    });

    test('should reject non-numeric levels', () => {
      const levels = { up: 'invalid', down: 3, left: 0, right: 5 } as any;
      const result = processor.validateLevels(levels);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain("Level 'up' must be a number");
    });
  });

  describe('Velocity Validation', () => {
    test('should validate correct velocities', () => {
      const result = processor.validateVelocity(0.5, -0.3);
      
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    test('should reject velocities exceeding VX_MAX', () => {
      const result = processor.validateVelocity(1.5, 0.0);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('vx magnitude 1.5 exceeds maximum 1');
    });

    test('should reject velocities exceeding VY_MAX', () => {
      const result = processor.validateVelocity(0.0, -1.0);
      
      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('vy magnitude 1 exceeds maximum 0.8');
    });

    test('should reject non-finite velocities', () => {
      const result1 = processor.validateVelocity(NaN, 0.0);
      const result2 = processor.validateVelocity(0.0, Infinity);
      
      expect(result1.isValid).toBe(false);
      expect(result1.errors).toContain('vx must be a finite number');
      
      expect(result2.isValid).toBe(false);
      expect(result2.errors).toContain('vy must be a finite number');
    });
  });

  describe('Command Validation', () => {
    test('should validate complete valid command', () => {
      const command: VelocityCommand = {
        vx: 0.3,
        vy: 0.16,
        levels: { up: 3, down: 0, left: 0, right: 2 }
      };
      
      const result = processor.validateCommand(command);
      expect(result.isValid).toBe(true);
      expect(result.errors).toHaveLength(0);
    });

    test('should detect velocity-level mismatch', () => {
      const command: VelocityCommand = {
        vx: 0.5, // Should be 0.3 for levels
        vy: 0.16,
        levels: { up: 3, down: 0, left: 0, right: 2 }
      };
      
      const result = processor.validateCommand(command);
      expect(result.isValid).toBe(false);
      expect(result.errors.some(e => e.includes('vx 0.5 does not match calculated velocity'))).toBe(true);
    });

    test('should accumulate multiple validation errors', () => {
      const command: VelocityCommand = {
        vx: 2.0, // Exceeds VX_MAX
        vy: 0.16,
        levels: { up: -1, down: 0, left: 0, right: 2 } // Invalid level
      };
      
      const result = processor.validateCommand(command);
      expect(result.isValid).toBe(false);
      expect(result.errors.length).toBeGreaterThan(1);
    });
  });

  describe('Command Creation', () => {
    test('should create valid command from levels', () => {
      const levels: SpeedLevels = { up: 4, down: 1, left: 2, right: 5 };
      const command = processor.createCommand(levels, 'test');
      
      expect(command.vx).toBe((4 - 1) * 0.1); // 0.3
      expect(command.vy).toBe((5 - 2) * 0.08); // 0.24
      expect(command.levels).toEqual(levels);
      expect(command.source).toBe('test');
      expect(command.timestamp).toBeInstanceOf(Date);
    });

    test('should use default source when not provided', () => {
      const levels: SpeedLevels = { up: 0, down: 0, left: 0, right: 0 };
      const command = processor.createCommand(levels);
      
      expect(command.source).toBe('web');
    });
  });

  describe('ROS2 Twist Message Conversion', () => {
    // Requirement 5.7: linear.x = vel_x, linear.y = vel_y, angular.z = 0
    test('should convert to correct Twist message format', () => {
      const command: VelocityCommand = {
        vx: 0.5,
        vy: -0.3,
        levels: { up: 5, down: 0, left: 0, right: 0 }
      };
      
      const twist = processor.toTwistMessage(command);
      
      expect(twist.linear.x).toBe(0.5);
      expect(twist.linear.y).toBe(-0.3);
      expect(twist.linear.z).toBe(0.0);
      expect(twist.angular.x).toBe(0.0);
      expect(twist.angular.y).toBe(0.0);
      expect(twist.angular.z).toBe(0.0);
    });
  });

  describe('Velocity Limits', () => {
    test('should return correct velocity limits', () => {
      const limits = processor.getVelocityLimits();
      
      expect(limits.vxMax).toBe(1.0);
      expect(limits.vyMax).toBe(0.8);
      expect(limits.stepX).toBe(0.1);
      expect(limits.stepY).toBe(0.08);
    });
  });
});

describe('VelocityUtils', () => {
  describe('Stop Command', () => {
    test('should create stop command with all zeros', () => {
      const stopLevels = VelocityUtils.createStopCommand();
      
      expect(stopLevels.up).toBe(0);
      expect(stopLevels.down).toBe(0);
      expect(stopLevels.left).toBe(0);
      expect(stopLevels.right).toBe(0);
    });
  });

  describe('Level Increment/Decrement', () => {
    test('should increment level correctly', () => {
      expect(VelocityUtils.incrementLevel(5)).toBe(6);
      expect(VelocityUtils.incrementLevel(9)).toBe(10);
      expect(VelocityUtils.incrementLevel(10)).toBe(10); // Clamped to max
    });

    test('should decrement level correctly', () => {
      expect(VelocityUtils.decrementLevel(5)).toBe(4);
      expect(VelocityUtils.decrementLevel(1)).toBe(0);
      expect(VelocityUtils.decrementLevel(0)).toBe(0); // Clamped to min
    });
  });

  describe('Stopped State Detection', () => {
    test('should detect stopped state', () => {
      const stoppedLevels: SpeedLevels = { up: 0, down: 0, left: 0, right: 0 };
      const movingLevels: SpeedLevels = { up: 1, down: 0, left: 0, right: 0 };
      
      expect(VelocityUtils.isStopped(stoppedLevels)).toBe(true);
      expect(VelocityUtils.isStopped(movingLevels)).toBe(false);
    });
  });
});

describe('Edge Cases and Error Conditions', () => {
  let processor: VelocityProcessor;

  beforeEach(() => {
    processor = new VelocityProcessor();
  });

  test('should handle maximum opposing levels', () => {
    const levels: SpeedLevels = { up: 10, down: 10, left: 10, right: 10 };
    const { vx, vy } = processor.calculateVelocity(levels);
    
    expect(vx).toBe(0); // 10 - 10 = 0
    expect(vy).toBe(0); // 10 - 10 = 0
  });

  test('should handle zero levels', () => {
    const levels: SpeedLevels = { up: 0, down: 0, left: 0, right: 0 };
    const { vx, vy } = processor.calculateVelocity(levels);
    
    expect(vx).toBe(0);
    expect(vy).toBe(0);
  });

  test('should handle floating point precision', () => {
    const levels: SpeedLevels = { up: 1, down: 0, left: 0, right: 1 };
    const { vx, vy } = processor.calculateVelocity(levels);
    
    // Should handle small floating point differences
    expect(vx).toBeCloseTo(0.1, 10);
    expect(vy).toBeCloseTo(0.08, 10);
  });
});