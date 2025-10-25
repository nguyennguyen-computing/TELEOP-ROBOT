import type { VelocityCommand, SpeedLevels, ValidationResult } from '@web-teleop-robot/shared';
import { getEnvironmentConfig } from '../config/environment';

/**
 * Velocity processor class that handles all velocity-related calculations
 */
export class VelocityProcessor {
  private readonly vxMax: number;
  private readonly vyMax: number;
  private readonly stepX: number;
  private readonly stepY: number;

  constructor() {
    const config = getEnvironmentConfig();
    this.vxMax = config.VX_MAX;
    this.vyMax = config.VY_MAX;
    this.stepX = this.vxMax / 10; // STEP_X = VX_MAX / 10
    this.stepY = this.vyMax / 10; // STEP_Y = VY_MAX / 10
  }

  private calculateNetLevels(levels: SpeedLevels): { netX: number; netY: number } {
    const netX = levels.up - levels.down;     // Forward(+) - Backward(-)
    const netY = levels.right - levels.left;  // Right(+) - Left(-)
    
    return { netX, netY };
  }

  /**
   * Clamps a value between min and max bounds
   */
  private clamp(value: number, min: number, max: number): number {
    return Math.min(Math.max(value, min), max);
  }

  public calculateVelocity(levels: SpeedLevels): { vx: number; vy: number } {
    // Calculate net levels
    const { netX, netY } = this.calculateNetLevels(levels);
    
    // Clamp net levels to valid range (-10 to +10)
    const clampedNetX = this.clamp(netX, -10, 10);
    const clampedNetY = this.clamp(netY, -10, 10);
    
    // Convert to velocity using step scaling
    const vx = clampedNetX * this.stepX;  // Forward/Backward velocity
    const vy = clampedNetY * this.stepY;  // Left/Right velocity
    
    return { vx, vy };
  }

  /**
   * Validates speed levels are within acceptable range (0-10)
   */
  public validateLevels(levels: SpeedLevels): ValidationResult {
    const errors: string[] = [];
    
    // Check each direction level
    const directions = ['up', 'down', 'left', 'right'] as const;
    
    for (const direction of directions) {
      const level = levels[direction];
      
      if (typeof level !== 'number') {
        errors.push(`Level '${direction}' must be a number`);
        continue;
      }
      
      if (!Number.isInteger(level)) {
        errors.push(`Level '${direction}' must be an integer`);
        continue;
      }
      
      if (level < 0 || level > 10) {
        errors.push(`Level '${direction}' must be between 0 and 10, got ${level}`);
      }
    }
    
    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * Validates velocity values are within acceptable ranges
   */
  public validateVelocity(vx: number, vy: number): ValidationResult {
    const errors: string[] = [];
    
    // Check vx is a valid number
    if (typeof vx !== 'number' || !Number.isFinite(vx)) {
      errors.push('vx must be a finite number');
    } else if (Math.abs(vx) > this.vxMax) {
      errors.push(`vx magnitude ${Math.abs(vx)} exceeds maximum ${this.vxMax}`);
    }
    
    // Check vy is a valid number
    if (typeof vy !== 'number' || !Number.isFinite(vy)) {
      errors.push('vy must be a finite number');
    } else if (Math.abs(vy) > this.vyMax) {
      errors.push(`vy magnitude ${Math.abs(vy)} exceeds maximum ${this.vyMax}`);
    }
    
    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * Validates a complete velocity command
   */
  public validateCommand(command: VelocityCommand): ValidationResult {
    const errors: string[] = [];
    
    // Validate levels
    const levelValidation = this.validateLevels(command.levels);
    if (!levelValidation.isValid) {
      errors.push(...levelValidation.errors);
    }
    
    // Validate velocity values
    const velocityValidation = this.validateVelocity(command.vx, command.vy);
    if (!velocityValidation.isValid) {
      errors.push(...velocityValidation.errors);
    }
    
    // Cross-validate: check if velocity matches calculated velocity from levels
    if (levelValidation.isValid) {
      const calculatedVelocity = this.calculateVelocity(command.levels);
      const tolerance = 0.001; // Small tolerance for floating point comparison
      
      if (Math.abs(command.vx - calculatedVelocity.vx) > tolerance) {
        errors.push(`vx ${command.vx} does not match calculated velocity ${calculatedVelocity.vx} from levels`);
      }
      
      if (Math.abs(command.vy - calculatedVelocity.vy) > tolerance) {
        errors.push(`vy ${command.vy} does not match calculated velocity ${calculatedVelocity.vy} from levels`);
      }
    }
    
    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * Creates a velocity command from speed levels
   */
  public createCommand(levels: SpeedLevels, source: string = 'web'): VelocityCommand {
    const { vx, vy } = this.calculateVelocity(levels);
    
    return {
      vx,
      vy,
      levels,
      timestamp: new Date(),
      source
    };
  }

  public toTwistMessage(command: VelocityCommand) {
    return {
      linear: {
        x: command.vx,  // Forward/Backward velocity
        y: command.vy,  // Left/Right velocity  
        z: 0.0          // No vertical movement
      },
      angular: {
        x: 0.0,         // No roll
        y: 0.0,         // No pitch
        z: 0.0          // No yaw rotation
      }
    };
  }

  /**
   * Gets the current velocity limits and step sizes
   */
  public getVelocityLimits() {
    return {
      vxMax: this.vxMax,
      vyMax: this.vyMax,
      stepX: this.stepX,
      stepY: this.stepY
    };
  }
}

/**
 * Singleton instance of the velocity processor
 */
let _velocityProcessor: VelocityProcessor | null = null;

export const velocityProcessor = {
  get instance(): VelocityProcessor {
    if (!_velocityProcessor) {
      _velocityProcessor = new VelocityProcessor();
    }
    return _velocityProcessor;
  },
  
  // Delegate methods to the instance
  validateCommand: (command: any) => velocityProcessor.instance.validateCommand(command),
  calculateVelocity: (levels: any) => velocityProcessor.instance.calculateVelocity(levels),
  createCommand: (levels: any, source?: string) => velocityProcessor.instance.createCommand(levels, source),
  toTwistMessage: (command: any) => velocityProcessor.instance.toTwistMessage(command),
  getVelocityLimits: () => velocityProcessor.instance.getVelocityLimits(),
};

/**
 * Utility functions for common velocity operations
 */
export const VelocityUtils = {
  /**
   * Creates a stop command (all levels set to 0)
   */
  createStopCommand(): SpeedLevels {
    return {
      up: 0,
      down: 0,
      left: 0,
      right: 0
    };
  },

  /**
   * Increments a level by 1, clamped to maximum of 10
   */
  incrementLevel(currentLevel: number): number {
    return Math.min(currentLevel + 1, 10);
  },

  /**
   * Decrements a level by 1, clamped to minimum of 0
   */
  decrementLevel(currentLevel: number): number {
    return Math.max(currentLevel - 1, 0);
  },

  /**
   * Checks if all levels are zero (stopped state)
   */
  isStopped(levels: SpeedLevels): boolean {
    return levels.up === 0 && levels.down === 0 && 
           levels.left === 0 && levels.right === 0;
  }
};