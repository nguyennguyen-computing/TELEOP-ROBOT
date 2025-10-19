/**
 * Error handling middleware for Express.js application
 */

import { Request, Response, NextFunction } from 'express';
import { ApiResponse } from '@web-teleop-robot/shared';

/**
 * Custom error class for API errors
 */
export class ApiError extends Error {
  public statusCode: number;
  public isOperational: boolean;

  constructor(message: string, statusCode: number = 500, isOperational: boolean = true) {
    super(message);
    this.statusCode = statusCode;
    this.isOperational = isOperational;
    
    // Maintains proper stack trace for where our error was thrown
    Error.captureStackTrace(this, this.constructor);
  }
}

/**
 * Creates a standardized API error response
 */
function createErrorResponse(error: Error): ApiResponse {
  return {
    ok: false,
    error: error.message,
    timestamp: new Date().toISOString(),
  };
}

/**
 * Global error handling middleware
 * Must be the last middleware in the chain
 */
export function errorHandler(
  error: Error,
  req: Request,
  res: Response,
  _next: NextFunction
): void {
  // Log error details
  console.error('Error occurred:', {
    message: error.message,
    stack: error.stack,
    url: req.url,
    method: req.method,
    ip: req.ip,
    userAgent: req.get('User-Agent'),
    timestamp: new Date().toISOString(),
  });

  // Handle different types of errors
  let statusCode = 500;
  let message = 'Internal Server Error';

  if (error instanceof ApiError) {
    statusCode = error.statusCode;
    message = error.message;
  } else if (error.name === 'ValidationError') {
    statusCode = 400;
    message = error.message;
  } else if (error.name === 'CastError') {
    statusCode = 400;
    message = 'Invalid data format';
  } else if (error.name === 'MongoError' || error.name === 'MongoServerError') {
    statusCode = 503;
    message = 'Database service unavailable';
  } else if (error.name === 'SyntaxError' && 'body' in error) {
    statusCode = 400;
    message = 'Invalid JSON in request body';
  } else if (error.message.includes('ECONNREFUSED')) {
    statusCode = 503;
    message = 'External service unavailable';
  }

  // Don't expose internal error details in production
  if (process.env.NODE_ENV === 'production' && statusCode === 500) {
    message = 'Internal Server Error';
  }

  // Send error response
  const errorResponse = createErrorResponse(new Error(message));
  res.status(statusCode).json(errorResponse);
}

/**
 * 404 Not Found handler
 */
export function notFoundHandler(req: Request, res: Response): void {
  const errorResponse: ApiResponse = {
    ok: false,
    error: `Route ${req.method} ${req.path} not found`,
    timestamp: new Date().toISOString(),
  };

  res.status(404).json(errorResponse);
}

/**
 * Async error wrapper for route handlers
 * Catches async errors and passes them to the error handler
 */
export function asyncHandler(
  fn: (req: Request, res: Response, next: NextFunction) => Promise<any>
) {
  return (req: Request, res: Response, next: NextFunction) => {
    Promise.resolve(fn(req, res, next)).catch(next);
  };
}

/**
 * Validation error helper
 */
export function createValidationError(message: string): ApiError {
  return new ApiError(message, 400);
}

/**
 * Not found error helper
 */
export function createNotFoundError(resource: string): ApiError {
  return new ApiError(`${resource} not found`, 404);
}

/**
 * Service unavailable error helper
 */
export function createServiceUnavailableError(service: string): ApiError {
  return new ApiError(`${service} service unavailable`, 503);
}