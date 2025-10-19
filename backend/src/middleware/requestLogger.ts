/**
 * Custom request logging middleware for structured logging
 */

import { Request, Response, NextFunction } from 'express';
import { v4 as uuidv4 } from 'uuid';

// Extend Request interface to include correlation ID
declare global {
  namespace Express {
    interface Request {
      correlationId?: string;
      startTime?: number;
    }
  }
}

/**
 * Structured log entry interface
 */
interface LogEntry {
  timestamp: string;
  level: 'info' | 'warn' | 'error';
  service: string;
  event: string;
  correlationId: string;
  data: {
    method: string;
    url: string;
    userAgent?: string | undefined;
    ip: string;
    duration?: number;
    statusCode?: number;
    contentLength?: number;
    message?: string;
  };
  error?: string;
}

/**
 * Creates a structured log entry
 */
function createLogEntry(
  level: LogEntry['level'],
  event: string,
  req: Request,
  res?: Response,
  error?: Error
): LogEntry {
  const entry: LogEntry = {
    timestamp: new Date().toISOString(),
    level,
    service: 'node-api',
    event,
    correlationId: req.correlationId || 'unknown',
    data: {
      method: req.method,
      url: req.url,
      userAgent: req.get('User-Agent'),
      ip: req.ip || req.connection.remoteAddress || 'unknown',
    },
  };

  if (res) {
    entry.data.statusCode = res.statusCode;
    entry.data.contentLength = parseInt(res.get('Content-Length') || '0', 10);
  }

  if (req.startTime) {
    entry.data.duration = Date.now() - req.startTime;
  }

  if (error) {
    entry.error = error.message;
  }

  return entry;
}

/**
 * Logs a structured entry to console
 */
function logEntry(entry: LogEntry): void {
  const logMessage = JSON.stringify(entry);
  
  switch (entry.level) {
    case 'error':
      console.error(logMessage);
      break;
    case 'warn':
      console.warn(logMessage);
      break;
    default:
      console.log(logMessage);
  }
}

/**
 * Request logging middleware
 * Adds correlation ID and logs request/response details
 */
export function requestLogger(req: Request, res: Response, next: NextFunction): void {
  // Generate correlation ID for request tracking
  req.correlationId = uuidv4();
  req.startTime = Date.now();

  // Log incoming request
  const requestEntry = createLogEntry('info', 'request_received', req);
  logEntry(requestEntry);

  // Override res.end to log response
  const originalEnd = res.end.bind(res);
  res.end = function(chunk?: any, encoding?: any) {
    // Log response
    const responseEntry = createLogEntry('info', 'request_completed', req, res);
    logEntry(responseEntry);

    // Call original end method
    return originalEnd(chunk, encoding);
  };

  // Add correlation ID to response headers for debugging
  res.setHeader('X-Correlation-ID', req.correlationId || 'unknown');

  next();
}

/**
 * Logs an error with request context
 */
export function logError(req: Request, error: Error): void {
  const errorEntry = createLogEntry('error', 'request_error', req, undefined, error);
  logEntry(errorEntry);
}

/**
 * Logs a warning with request context
 */
export function logWarning(req: Request, message: string): void {
  const warningEntry = createLogEntry('warn', 'request_warning', req);
  warningEntry.data = { ...warningEntry.data, message };
  logEntry(warningEntry);
}

/**
 * Logs custom application events
 */
export function logEvent(
  event: string,
  data: Record<string, any>,
  level: LogEntry['level'] = 'info'
): void {
  const entry: Omit<LogEntry, 'data'> & { data: Record<string, any> } = {
    timestamp: new Date().toISOString(),
    level,
    service: 'node-api',
    event,
    correlationId: 'system',
    data,
  };

  logEntry(entry as LogEntry);
}