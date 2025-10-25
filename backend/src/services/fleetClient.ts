import axios, { AxiosInstance, AxiosError, AxiosRequestConfig } from 'axios';
import { VelocityCommand } from '@web-teleop-robot/shared';
import { getEnvironmentConfig } from '../config/environment';

/**
 * Fleet API response interface
 */
export interface FleetVelocityResponse {
  published: boolean;
  timestamp?: string;
  message?: string;
}

/**
 * Fleet API request payload
 */
export interface FleetVelocityRequest {
  vx: number;
  vy: number;
  levels?: {
    up: number;
    down: number;
    left: number;
    right: number;
  };
}

/**
 * Circuit breaker states
 */
export enum CircuitBreakerState {
  CLOSED = 'CLOSED',
  OPEN = 'OPEN',
  HALF_OPEN = 'HALF_OPEN'
}

/**
 * Circuit breaker configuration
 */
export interface CircuitBreakerConfig {
  failureThreshold: number;
  recoveryTimeout: number;
  monitoringPeriod: number;
  expectedErrors: number[];
}

/**
 * Retry configuration
 */
export interface RetryConfig {
  maxRetries: number;
  baseDelay: number;
  maxDelay: number;
  backoffMultiplier: number;
  retryableStatusCodes: number[];
}

/**
 * Fleet client statistics
 */
export interface FleetClientStats {
  totalRequests: number;
  successfulRequests: number;
  failedRequests: number;
  circuitBreakerState: CircuitBreakerState;
  lastRequestTime: Date | null;
  lastSuccessTime: Date | null;
  lastFailureTime: Date | null;
  averageResponseTime: number;
  consecutiveFailures: number;
}

/**
 * Fleet Server HTTP Client with retry logic and circuit breaker pattern
 */
export class FleetClient {
  private readonly httpClient: AxiosInstance;
  private readonly retryConfig: RetryConfig;
  private readonly circuitBreakerConfig: CircuitBreakerConfig;
  
  // Circuit breaker state
  private circuitBreakerState: CircuitBreakerState = CircuitBreakerState.CLOSED;
  private consecutiveFailures = 0;
  private nextRetryTime: Date | null = null;
  
  // Statistics
  private stats: FleetClientStats = {
    totalRequests: 0,
    successfulRequests: 0,
    failedRequests: 0,
    circuitBreakerState: CircuitBreakerState.CLOSED,
    lastRequestTime: null,
    lastSuccessTime: null,
    lastFailureTime: null,
    averageResponseTime: 0,
    consecutiveFailures: 0
  };
  
  // Response time tracking
  private responseTimes: number[] = [];
  private readonly maxResponseTimeHistory = 100;

  constructor(
    baseURL?: string,
    retryConfig?: Partial<RetryConfig>,
    circuitBreakerConfig?: Partial<CircuitBreakerConfig>
  ) {
    const config = getEnvironmentConfig();
    const fleetApiUrl = baseURL || config.FLEET_API_URL;

    // Default retry configuration
    this.retryConfig = {
      maxRetries: 3,
      baseDelay: 1000, // 1 second
      maxDelay: 10000, // 10 seconds
      backoffMultiplier: 2,
      retryableStatusCodes: [408, 429, 500, 502, 503, 504],
      ...retryConfig
    };

    // Default circuit breaker configuration
    this.circuitBreakerConfig = {
      failureThreshold: 5,
      recoveryTimeout: 30000, // 30 seconds
      monitoringPeriod: 60000, // 1 minute
      expectedErrors: [400, 401, 403, 404, 422], // Don't count these as circuit breaker failures
      ...circuitBreakerConfig
    };

    // Prepare authentication headers
    const authHeaders: Record<string, string> = {};
    if (config.AUTH_ENABLED && config.API_KEYS) {
      // Use the first API key for fleet communication
      const apiKey = config.API_KEYS.split(',')[0].trim();
      if (apiKey) {
        authHeaders['Authorization'] = `ApiKey ${apiKey}`;
        console.log('Fleet API authentication enabled with API key');
      }
    }

    // Create axios instance
    this.httpClient = axios.create({
      baseURL: fleetApiUrl,
      timeout: 10000, // 10 second timeout
      headers: {
        'Content-Type': 'application/json',
        'Accept': 'application/json',
        'User-Agent': 'web-teleop-robot-backend/1.0.0',
        ...authHeaders
      }
    });

    // Add request interceptor for logging
    this.httpClient.interceptors.request.use(
      (config) => {
        console.log(`Fleet API Request: ${config.method?.toUpperCase()} ${config.url}`);
        return config;
      },
      (error) => {
        console.error('Fleet API Request Error:', error);
        return Promise.reject(error);
      }
    );

    // Add response interceptor for logging and stats
    this.httpClient.interceptors.response.use(
      (response) => {
        const responseTime = Date.now() - (response.config as any).requestStartTime;
        this.recordResponseTime(responseTime);
        console.log(`Fleet API Response: ${response.status} ${response.statusText} (${responseTime}ms)`);
        return response;
      },
      (error) => {
        if (error.response) {
          const responseTime = Date.now() - (error.config as any).requestStartTime;
          this.recordResponseTime(responseTime);
          console.error(`Fleet API Error Response: ${error.response.status} ${error.response.statusText} (${responseTime}ms)`);
        } else {
          console.error('Fleet API Network Error:', error.message);
        }
        return Promise.reject(error);
      }
    );
  }

  /**
   * Send velocity command to fleet server
   * @param command Velocity command to send
   * @returns Promise resolving to fleet response
   */
  async sendVelocityCommand(command: VelocityCommand): Promise<FleetVelocityResponse> {
    // Check circuit breaker state
    if (!this.isCircuitBreakerClosed()) {
      throw new Error(`Circuit breaker is ${this.circuitBreakerState}. Fleet server unavailable.`);
    }

    const payload: FleetVelocityRequest = {
      vx: Number(command.vx.toFixed(1)),
      vy: Number(command.vy.toFixed(1)),
      levels: command.levels
    };

    return this.executeWithRetry(async () => {
      const startTime = Date.now();
      
      try {
        this.stats.totalRequests++;
        this.stats.lastRequestTime = new Date();
        
        // Add request start time for response time calculation
        const config: AxiosRequestConfig = {
          ...({} as any),
          requestStartTime: startTime
        } as any;

        console.log('Sending velocity command to fleet server:', '/api/v1/fleet/vel');
        const response = await this.httpClient.post<FleetVelocityResponse>('/api/v1/fleet/vel', payload, config);
        
        // Record success
        this.recordSuccess();
        
        return response.data;
      } catch (error) {
        // Record failure
        this.recordFailure(error as AxiosError);
        throw error;
      }
    });
  }

  /**
   * Health check for fleet server
   * @returns Promise resolving to health status
   */
  async healthCheck(): Promise<{ healthy: boolean; responseTime: number; error?: string }> {
    const startTime = Date.now();
    
    try {
      const config: AxiosRequestConfig = {
        timeout: 5000, // Shorter timeout for health checks
        ...({} as any),
        requestStartTime: startTime
      } as any;

      await this.httpClient.get('/health', config);
      const responseTime = Date.now() - startTime;
      
      return { healthy: true, responseTime };
    } catch (error) {
      const responseTime = Date.now() - startTime;
      const errorMessage = error instanceof Error ? error.message : 'Unknown error';
      
      return { healthy: false, responseTime, error: errorMessage };
    }
  }

  /**
   * Get client statistics
   * @returns Current client statistics
   */
  getStats(): FleetClientStats {
    return {
      ...this.stats,
      circuitBreakerState: this.circuitBreakerState,
      consecutiveFailures: this.consecutiveFailures,
      averageResponseTime: this.calculateAverageResponseTime()
    };
  }

  /**
   * Reset circuit breaker and statistics
   */
  reset(): void {
    this.circuitBreakerState = CircuitBreakerState.CLOSED;
    this.consecutiveFailures = 0;
    this.nextRetryTime = null;
    this.responseTimes = [];
    
    this.stats = {
      totalRequests: 0,
      successfulRequests: 0,
      failedRequests: 0,
      circuitBreakerState: CircuitBreakerState.CLOSED,
      lastRequestTime: null,
      lastSuccessTime: null,
      lastFailureTime: null,
      averageResponseTime: 0,
      consecutiveFailures: 0
    };
  }

  /**
   * Execute request with retry logic
   */
  private async executeWithRetry<T>(operation: () => Promise<T>): Promise<T> {
    let lastError: Error;
    
    for (let attempt = 0; attempt <= this.retryConfig.maxRetries; attempt++) {
      try {
        return await operation();
      } catch (error) {
        lastError = error as Error;
        
        // Don't retry on the last attempt
        if (attempt === this.retryConfig.maxRetries) {
          break;
        }
        
        // Check if error is retryable
        if (!this.isRetryableError(error as AxiosError)) {
          break;
        }
        
        // Calculate delay with exponential backoff
        const delay = Math.min(
          this.retryConfig.baseDelay * Math.pow(this.retryConfig.backoffMultiplier, attempt),
          this.retryConfig.maxDelay
        );
        
        console.log(`Fleet API retry attempt ${attempt + 1}/${this.retryConfig.maxRetries} after ${delay}ms`);
        await this.sleep(delay);
      }
    }
    
    throw lastError!;
  }

  /**
   * Check if error is retryable
   */
  private isRetryableError(error: AxiosError): boolean {
    // Network errors are always retryable
    if (!error.response) {
      return true;
    }
    
    // Check if status code is retryable
    return this.retryConfig.retryableStatusCodes.includes(error.response.status);
  }

  /**
   * Check if circuit breaker allows requests
   */
  private isCircuitBreakerClosed(): boolean {
    const now = new Date();
    
    switch (this.circuitBreakerState) {
      case CircuitBreakerState.CLOSED:
        return true;
        
      case CircuitBreakerState.OPEN:
        // Check if recovery timeout has passed
        if (this.nextRetryTime && now >= this.nextRetryTime) {
          this.circuitBreakerState = CircuitBreakerState.HALF_OPEN;
          console.log('Circuit breaker transitioning to HALF_OPEN state');
          return true;
        }
        return false;
        
      case CircuitBreakerState.HALF_OPEN:
        return true;
        
      default:
        return false;
    }
  }

  /**
   * Record successful request
   */
  private recordSuccess(): void {
    this.stats.successfulRequests++;
    this.stats.lastSuccessTime = new Date();
    this.consecutiveFailures = 0;
    
    // Close circuit breaker if it was half-open
    if (this.circuitBreakerState === CircuitBreakerState.HALF_OPEN) {
      this.circuitBreakerState = CircuitBreakerState.CLOSED;
      console.log('Circuit breaker closed after successful request');
    }
  }

  /**
   * Record failed request
   */
  private recordFailure(error: AxiosError): void {
    this.stats.failedRequests++;
    this.stats.lastFailureTime = new Date();
    
    // Don't count expected errors as circuit breaker failures
    if (error.response && this.circuitBreakerConfig.expectedErrors.includes(error.response.status)) {
      return;
    }
    
    this.consecutiveFailures++;
    
    // Check if we should open the circuit breaker
    if (this.consecutiveFailures >= this.circuitBreakerConfig.failureThreshold) {
      this.circuitBreakerState = CircuitBreakerState.OPEN;
      this.nextRetryTime = new Date(Date.now() + this.circuitBreakerConfig.recoveryTimeout);
      console.log(`Circuit breaker opened after ${this.consecutiveFailures} consecutive failures. Next retry at ${this.nextRetryTime.toISOString()}`);
    }
  }

  /**
   * Record response time
   */
  private recordResponseTime(responseTime: number): void {
    this.responseTimes.push(responseTime);
    
    // Keep only the last N response times
    if (this.responseTimes.length > this.maxResponseTimeHistory) {
      this.responseTimes.shift();
    }
  }

  /**
   * Calculate average response time
   */
  private calculateAverageResponseTime(): number {
    if (this.responseTimes.length === 0) {
      return 0;
    }
    
    const sum = this.responseTimes.reduce((acc, time) => acc + time, 0);
    return Math.round(sum / this.responseTimes.length);
  }

  /**
   * Sleep for specified milliseconds
   */
  private sleep(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms));
  }
}

/**
 * Default fleet client instance
 * Created lazily to avoid issues during testing
 */
let _fleetClient: FleetClient | null = null;

export const fleetClient = {
  sendVelocityCommand: (command: VelocityCommand) => {
    if (!_fleetClient) {
      _fleetClient = new FleetClient();
    }
    return _fleetClient.sendVelocityCommand(command);
  },
  
  healthCheck: () => {
    if (!_fleetClient) {
      _fleetClient = new FleetClient();
    }
    return _fleetClient.healthCheck();
  },
  
  getStats: () => {
    if (!_fleetClient) {
      _fleetClient = new FleetClient();
    }
    return _fleetClient.getStats();
  },
  
  reset: () => {
    if (!_fleetClient) {
      _fleetClient = new FleetClient();
    }
    return _fleetClient.reset();
  }
};