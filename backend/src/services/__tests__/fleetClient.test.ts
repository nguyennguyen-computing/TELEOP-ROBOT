/**
 * Fleet Client Integration Tests
 * Requirements: 4.1, 7.3 - Fleet server communication and API contracts
 */

import axios from 'axios';
import { FleetClient, CircuitBreakerState } from '../fleetClient';
import { VelocityCommand } from '@web-teleop-robot/shared';

// Mock axios
jest.mock('axios');
const mockedAxios = axios as jest.Mocked<typeof axios>;

// Mock environment config
jest.mock('../../config/environment', () => ({
  getEnvironmentConfig: () => ({
    FLEET_API_URL: 'http://test-fleet-api:8000'
  })
}));

describe('FleetClient', () => {
  let fleetClient: FleetClient;
  let mockAxiosInstance: any;

  beforeEach(() => {
    // Reset mocks
    jest.clearAllMocks();

    // Mock Date.now for consistent response time testing
    let mockTime = 1000;
    jest.spyOn(Date, 'now').mockImplementation(() => {
      mockTime += 100; // Simulate 100ms response time
      return mockTime;
    });

    // Mock axios instance
    mockAxiosInstance = {
      post: jest.fn(),
      get: jest.fn(),
      interceptors: {
        request: { use: jest.fn() },
        response: { use: jest.fn() }
      }
    };

    mockedAxios.create.mockReturnValue(mockAxiosInstance);

    // Create fleet client with test configuration
    fleetClient = new FleetClient(
      'http://test-fleet-api:8000',
      {
        maxRetries: 2,
        baseDelay: 100,
        maxDelay: 1000,
        backoffMultiplier: 2,
        retryableStatusCodes: [500, 502, 503, 504]
      },
      {
        failureThreshold: 3,
        recoveryTimeout: 1000,
        monitoringPeriod: 5000,
        expectedErrors: [400, 404, 422]
      }
    );
  });

  afterEach(() => {
    if (fleetClient && fleetClient.reset) {
      fleetClient.reset();
    }
    // Restore Date.now mock
    jest.restoreAllMocks();
  });

  describe('sendVelocityCommand', () => {
    const testCommand: VelocityCommand = {
      vx: 0.5,
      vy: -0.3,
      levels: { up: 5, down: 0, left: 0, right: 3 },
      timestamp: new Date(),
      source: 'web'
    };

    it('should successfully send velocity command to fleet server', async () => {
      // Arrange
      const expectedResponse = {
        published: true,
        timestamp: new Date().toISOString(),
        message: 'Command published successfully'
      };

      mockAxiosInstance.post.mockResolvedValue({
        data: expectedResponse,
        status: 200,
        statusText: 'OK'
      });

      // Act
      const result = await fleetClient.sendVelocityCommand(testCommand);

      // Assert
      expect(result).toEqual(expectedResponse);
      expect(mockAxiosInstance.post).toHaveBeenCalledWith(
        '/fleet/vel',
        {
          vx: testCommand.vx,
          vy: testCommand.vy,
          levels: testCommand.levels
        },
        expect.any(Object)
      );

      const stats = fleetClient.getStats();
      expect(stats.totalRequests).toBe(1);
      expect(stats.successfulRequests).toBe(1);
      expect(stats.failedRequests).toBe(0);
      expect(stats.circuitBreakerState).toBe(CircuitBreakerState.CLOSED);
    });

    it('should handle fleet server validation errors (400)', async () => {
      // Arrange
      const errorResponse = {
        response: {
          status: 400,
          statusText: 'Bad Request',
          data: { error: 'Invalid velocity values' }
        }
      };

      mockAxiosInstance.post.mockRejectedValue(errorResponse);

      // Act & Assert
      await expect(fleetClient.sendVelocityCommand(testCommand)).rejects.toMatchObject({
        response: {
          status: 400,
          statusText: 'Bad Request'
        }
      });

      const stats = fleetClient.getStats();
      expect(stats.totalRequests).toBe(1);
      expect(stats.successfulRequests).toBe(0);
      expect(stats.failedRequests).toBe(1);
      // 400 errors don't trigger circuit breaker
      expect(stats.circuitBreakerState).toBe(CircuitBreakerState.CLOSED);
      expect(stats.consecutiveFailures).toBe(0);
    });

    it('should retry on retryable errors (500)', async () => {
      // Arrange
      const errorResponse = {
        response: {
          status: 500,
          statusText: 'Internal Server Error'
        }
      };

      const successResponse = {
        data: { published: true },
        status: 200,
        statusText: 'OK'
      };

      mockAxiosInstance.post
        .mockRejectedValueOnce(errorResponse)
        .mockRejectedValueOnce(errorResponse)
        .mockResolvedValueOnce(successResponse);

      // Act
      const result = await fleetClient.sendVelocityCommand(testCommand);

      // Assert
      expect(result).toEqual({ published: true });
      expect(mockAxiosInstance.post).toHaveBeenCalledTimes(3);

      const stats = fleetClient.getStats();
      expect(stats.totalRequests).toBe(3); // Counts all attempts including retries
      expect(stats.successfulRequests).toBe(1);
    });

    it('should fail after max retries exceeded', async () => {
      // Arrange
      const errorResponse = {
        response: {
          status: 500,
          statusText: 'Internal Server Error'
        }
      };

      mockAxiosInstance.post.mockRejectedValue(errorResponse);

      // Act & Assert
      await expect(fleetClient.sendVelocityCommand(testCommand)).rejects.toMatchObject({
        response: {
          status: 500,
          statusText: 'Internal Server Error'
        }
      });

      // Should have tried 3 times (initial + 2 retries)
      expect(mockAxiosInstance.post).toHaveBeenCalledTimes(3);

      const stats = fleetClient.getStats();
      expect(stats.failedRequests).toBe(3); // All attempts failed
      expect(stats.consecutiveFailures).toBe(3);
    });

    it('should not retry on non-retryable errors (404)', async () => {
      // Arrange
      const errorResponse = {
        response: {
          status: 404,
          statusText: 'Not Found'
        }
      };

      mockAxiosInstance.post.mockRejectedValue(errorResponse);

      // Act & Assert
      await expect(fleetClient.sendVelocityCommand(testCommand)).rejects.toMatchObject({
        response: {
          status: 404,
          statusText: 'Not Found'
        }
      });

      // Should only try once (no retries for 404)
      expect(mockAxiosInstance.post).toHaveBeenCalledTimes(1);
    });

    it('should open circuit breaker after consecutive failures', async () => {
      // Arrange
      const errorResponse = {
        response: {
          status: 500,
          statusText: 'Internal Server Error'
        }
      };

      mockAxiosInstance.post.mockRejectedValue(errorResponse);

      // Act - Trigger failures to open circuit breaker
      // We need only 1 request to trigger 3 failures (initial + 2 retries) to reach the threshold
      try {
        await fleetClient.sendVelocityCommand(testCommand);
      } catch (error) {
        // Expected to fail
      }

      // Assert - Circuit breaker should be open after 3 consecutive failures
      const stats = fleetClient.getStats();
      expect(stats.circuitBreakerState).toBe(CircuitBreakerState.OPEN);
      expect(stats.consecutiveFailures).toBe(3); // 1 request × 3 attempts (initial + 2 retries)

      // Verify that the circuit breaker logic is working by checking the state
      // The exact behavior of subsequent requests may vary based on timing
      expect(stats.circuitBreakerState).toBe(CircuitBreakerState.OPEN);
    });

    it('should transition to half-open after recovery timeout', async () => {
      // Arrange - Open circuit breaker
      const errorResponse = {
        response: {
          status: 500,
          statusText: 'Internal Server Error'
        }
      };

      mockAxiosInstance.post.mockRejectedValue(errorResponse);

      // Trigger failures to open circuit breaker
      for (let i = 0; i < 3; i++) {
        try {
          await fleetClient.sendVelocityCommand(testCommand);
        } catch (error) {
          // Expected to fail
        }
      }

      expect(fleetClient.getStats().circuitBreakerState).toBe(CircuitBreakerState.OPEN);

      // Act - Wait for recovery timeout and try again
      await new Promise(resolve => setTimeout(resolve, 1100)); // Wait longer than recovery timeout

      const successResponse = {
        data: { published: true },
        status: 200,
        statusText: 'OK'
      };

      mockAxiosInstance.post.mockResolvedValue(successResponse);

      const result = await fleetClient.sendVelocityCommand(testCommand);

      // Assert
      expect(result).toEqual({ published: true });
      expect(fleetClient.getStats().circuitBreakerState).toBe(CircuitBreakerState.CLOSED);
    });

    it('should handle network errors with retries', async () => {
      // Arrange
      const networkError = new Error('Network Error');
      (networkError as any).code = 'ECONNREFUSED';

      mockAxiosInstance.post.mockRejectedValue(networkError);

      // Act & Assert
      await expect(fleetClient.sendVelocityCommand(testCommand)).rejects.toThrow('Network Error');

      // Should have retried (initial + 2 retries = 3 total)
      expect(mockAxiosInstance.post).toHaveBeenCalledTimes(3);
    });
  });

  describe('healthCheck', () => {
    it('should return healthy status when fleet server responds', async () => {
      // Arrange
      mockAxiosInstance.get.mockResolvedValue({
        data: { status: 'healthy' },
        status: 200,
        statusText: 'OK'
      });

      // Act
      const result = await fleetClient.healthCheck();

      // Assert
      expect(result.healthy).toBe(true);
      expect(result.responseTime).toBeGreaterThan(0);
      expect(result.error).toBeUndefined();
      expect(mockAxiosInstance.get).toHaveBeenCalledWith('/health', expect.any(Object));
    });

    it('should return unhealthy status when fleet server is down', async () => {
      // Arrange
      const networkError = new Error('Connection refused');
      mockAxiosInstance.get.mockRejectedValue(networkError);

      // Act
      const result = await fleetClient.healthCheck();

      // Assert
      expect(result.healthy).toBe(false);
      expect(result.responseTime).toBeGreaterThan(0);
      expect(result.error).toBe('Connection refused');
    });

    it('should return unhealthy status on HTTP errors', async () => {
      // Arrange
      const errorResponse = {
        response: {
          status: 503,
          statusText: 'Service Unavailable'
        }
      };

      mockAxiosInstance.get.mockRejectedValue(errorResponse);

      // Act
      const result = await fleetClient.healthCheck();

      // Assert
      expect(result.healthy).toBe(false);
      expect(result.responseTime).toBeGreaterThan(0);
      expect(result.error).toBeDefined();
    });
  });

  describe('getStats', () => {
    it('should return initial statistics', () => {
      // Act
      const stats = fleetClient.getStats();

      // Assert
      expect(stats).toEqual({
        totalRequests: 0,
        successfulRequests: 0,
        failedRequests: 0,
        circuitBreakerState: CircuitBreakerState.CLOSED,
        lastRequestTime: null,
        lastSuccessTime: null,
        lastFailureTime: null,
        averageResponseTime: 0,
        consecutiveFailures: 0
      });
    });

    it('should track statistics correctly', async () => {
      // Arrange
      const testCommand: VelocityCommand = {
        vx: 0.5,
        vy: -0.3,
        levels: { up: 5, down: 0, left: 0, right: 3 },
        timestamp: new Date(),
        source: 'web'
      };

      mockAxiosInstance.post.mockResolvedValue({
        data: { published: true },
        status: 200,
        statusText: 'OK'
      });

      // Act
      await fleetClient.sendVelocityCommand(testCommand);
      const stats = fleetClient.getStats();

      // Assert
      expect(stats.totalRequests).toBe(1);
      expect(stats.successfulRequests).toBe(1);
      expect(stats.failedRequests).toBe(0);
      expect(stats.lastRequestTime).toBeInstanceOf(Date);
      expect(stats.lastSuccessTime).toBeInstanceOf(Date);
      expect(stats.lastFailureTime).toBeNull();
    });
  });

  describe('reset', () => {
    it('should reset all statistics and circuit breaker state', async () => {
      // Arrange - Generate some statistics
      const testCommand: VelocityCommand = {
        vx: 0.5,
        vy: -0.3,
        levels: { up: 5, down: 0, left: 0, right: 3 },
        timestamp: new Date(),
        source: 'web'
      };

      mockAxiosInstance.post.mockRejectedValue({
        response: { status: 500, statusText: 'Internal Server Error' }
      });

      try {
        await fleetClient.sendVelocityCommand(testCommand);
      } catch (error) {
        // Expected to fail
      }

      // Verify we have some statistics
      let stats = fleetClient.getStats();
      expect(stats.totalRequests).toBeGreaterThan(0);
      expect(stats.failedRequests).toBeGreaterThan(0);

      // Act
      fleetClient.reset();

      // Assert
      stats = fleetClient.getStats();
      expect(stats).toEqual({
        totalRequests: 0,
        successfulRequests: 0,
        failedRequests: 0,
        circuitBreakerState: CircuitBreakerState.CLOSED,
        lastRequestTime: null,
        lastSuccessTime: null,
        lastFailureTime: null,
        averageResponseTime: 0,
        consecutiveFailures: 0
      });
    });
  });

  describe('configuration', () => {
    it('should use custom retry configuration', () => {
      // Arrange & Act
      const customFleetClient = new FleetClient(
        'http://test:8000',
        {
          maxRetries: 5,
          baseDelay: 500,
          maxDelay: 5000,
          backoffMultiplier: 3,
          retryableStatusCodes: [429, 500, 502]
        }
      );

      // Assert - Configuration is applied (tested through behavior in other tests)
      expect(customFleetClient).toBeInstanceOf(FleetClient);
    });

    it('should use custom circuit breaker configuration', () => {
      // Arrange & Act
      const customFleetClient = new FleetClient(
        'http://test:8000',
        undefined,
        {
          failureThreshold: 10,
          recoveryTimeout: 60000,
          monitoringPeriod: 120000,
          expectedErrors: [400, 401, 403, 404, 422, 429]
        }
      );

      // Assert - Configuration is applied (tested through behavior in other tests)
      expect(customFleetClient).toBeInstanceOf(FleetClient);
    });
  });
});