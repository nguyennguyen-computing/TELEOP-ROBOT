/**
 * Tests for error handling middleware
 */

import { Request, Response, NextFunction } from 'express';
import {
  ApiError,
  errorHandler,
  notFoundHandler,
  asyncHandler,
  createValidationError,
  createNotFoundError,
  createServiceUnavailableError,
} from '../errorHandler';

// Mock Express request and response objects
const mockRequest = () => ({
  url: '/test',
  path: '/test',
  method: 'GET',
  ip: '127.0.0.1',
  get: jest.fn().mockReturnValue('test-user-agent'),
}) as unknown as Request;

const mockResponse = () => {
  const res = {} as Response;
  res.status = jest.fn().mockReturnValue(res);
  res.json = jest.fn().mockReturnValue(res);
  return res;
};

const mockNext = jest.fn() as NextFunction;

describe('Error Handler Middleware', () => {
  let consoleSpy: jest.SpyInstance;

  beforeEach(() => {
    consoleSpy = jest.spyOn(console, 'error').mockImplementation();
    jest.clearAllMocks();
  });

  afterEach(() => {
    consoleSpy.mockRestore();
  });

  describe('ApiError class', () => {
    it('should create ApiError with default values', () => {
      const error = new ApiError('Test error');

      expect(error.message).toBe('Test error');
      expect(error.statusCode).toBe(500);
      expect(error.isOperational).toBe(true);
      expect(error.stack).toBeDefined();
    });

    it('should create ApiError with custom values', () => {
      const error = new ApiError('Custom error', 400, false);

      expect(error.message).toBe('Custom error');
      expect(error.statusCode).toBe(400);
      expect(error.isOperational).toBe(false);
    });
  });

  describe('errorHandler', () => {
    it('should handle ApiError correctly', () => {
      const req = mockRequest();
      const res = mockResponse();
      const error = new ApiError('Test API error', 400);

      errorHandler(error, req, res, mockNext);

      expect(res.status).toHaveBeenCalledWith(400);
      expect(res.json).toHaveBeenCalledWith({
        ok: false,
        error: 'Test API error',
        timestamp: expect.any(String),
      });
    });

    it('should handle ValidationError', () => {
      const req = mockRequest();
      const res = mockResponse();
      const error = new Error('Validation failed');
      error.name = 'ValidationError';

      errorHandler(error, req, res, mockNext);

      expect(res.status).toHaveBeenCalledWith(400);
      expect(res.json).toHaveBeenCalledWith({
        ok: false,
        error: 'Validation failed',
        timestamp: expect.any(String),
      });
    });

    it('should handle MongoDB errors', () => {
      const req = mockRequest();
      const res = mockResponse();
      const error = new Error('Database connection failed');
      error.name = 'MongoError';

      errorHandler(error, req, res, mockNext);

      expect(res.status).toHaveBeenCalledWith(503);
      expect(res.json).toHaveBeenCalledWith({
        ok: false,
        error: 'Database service unavailable',
        timestamp: expect.any(String),
      });
    });

    it('should handle JSON syntax errors', () => {
      const req = mockRequest();
      const res = mockResponse();
      const error = new Error('Unexpected token in JSON');
      error.name = 'SyntaxError';
      (error as any).body = true;

      errorHandler(error, req, res, mockNext);

      expect(res.status).toHaveBeenCalledWith(400);
      expect(res.json).toHaveBeenCalledWith({
        ok: false,
        error: 'Invalid JSON in request body',
        timestamp: expect.any(String),
      });
    });

    it('should handle connection refused errors', () => {
      const req = mockRequest();
      const res = mockResponse();
      const error = new Error('connect ECONNREFUSED 127.0.0.1:8000');

      errorHandler(error, req, res, mockNext);

      expect(res.status).toHaveBeenCalledWith(503);
      expect(res.json).toHaveBeenCalledWith({
        ok: false,
        error: 'External service unavailable',
        timestamp: expect.any(String),
      });
    });

    it('should handle generic errors', () => {
      const req = mockRequest();
      const res = mockResponse();
      const error = new Error('Generic error');

      errorHandler(error, req, res, mockNext);

      expect(res.status).toHaveBeenCalledWith(500);
      expect(res.json).toHaveBeenCalledWith({
        ok: false,
        error: 'Internal Server Error',
        timestamp: expect.any(String),
      });
    });

    it('should log error details', () => {
      const req = mockRequest();
      const res = mockResponse();
      const error = new Error('Test error');

      errorHandler(error, req, res, mockNext);

      expect(consoleSpy).toHaveBeenCalledWith('Error occurred:', expect.objectContaining({
        message: 'Test error',
        url: '/test',
        method: 'GET',
        ip: '127.0.0.1',
      }));
    });
  });

  describe('notFoundHandler', () => {
    it('should return 404 with proper error message', () => {
      const req = mockRequest();
      const res = mockResponse();

      notFoundHandler(req, res);

      expect(res.status).toHaveBeenCalledWith(404);
      expect(res.json).toHaveBeenCalledWith({
        ok: false,
        error: 'Route GET /test not found',
        timestamp: expect.any(String),
      });
    });
  });

  describe('asyncHandler', () => {
    it('should handle successful async operations', async () => {
      const asyncFn = jest.fn().mockResolvedValue('success');
      const wrappedFn = asyncHandler(asyncFn);
      const req = mockRequest();
      const res = mockResponse();

      await wrappedFn(req, res, mockNext);

      expect(asyncFn).toHaveBeenCalledWith(req, res, mockNext);
      expect(mockNext).not.toHaveBeenCalled();
    });

    it('should catch and forward async errors', async () => {
      const error = new Error('Async error');
      const asyncFn = jest.fn().mockRejectedValue(error);
      const wrappedFn = asyncHandler(asyncFn);
      const req = mockRequest();
      const res = mockResponse();

      await wrappedFn(req, res, mockNext);

      expect(asyncFn).toHaveBeenCalledWith(req, res, mockNext);
      expect(mockNext).toHaveBeenCalledWith(error);
    });
  });

  describe('Error helper functions', () => {
    it('should create validation error', () => {
      const error = createValidationError('Invalid input');

      expect(error).toBeInstanceOf(ApiError);
      expect(error.message).toBe('Invalid input');
      expect(error.statusCode).toBe(400);
    });

    it('should create not found error', () => {
      const error = createNotFoundError('User');

      expect(error).toBeInstanceOf(ApiError);
      expect(error.message).toBe('User not found');
      expect(error.statusCode).toBe(404);
    });

    it('should create service unavailable error', () => {
      const error = createServiceUnavailableError('Database');

      expect(error).toBeInstanceOf(ApiError);
      expect(error.message).toBe('Database service unavailable');
      expect(error.statusCode).toBe(503);
    });
  });
});