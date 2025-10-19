/**
 * Unit tests for authentication middleware
 */

import { Request, Response, NextFunction } from 'express';
import jwt from 'jsonwebtoken';
import { AuthenticatedRequest, JWTPayload } from '@web-teleop-robot/shared';

// Mock environment configuration
jest.mock('../../config/environment', () => ({
  getEnvironmentConfig: () => ({
    AUTH_ENABLED: true,
    API_KEYS: 'test-key-1,test-key-2,admin-key',
    JWT_SECRET: 'test-secret-key',
    JWT_EXPIRES_IN: '1h',
    RATE_LIMIT_WINDOW_MS: 900000,
    RATE_LIMIT_MAX_REQUESTS: 100,
  }),
}));

// Import after mocking
import { AuthService, requireAuth, createRateLimit, addAuthContext } from '../auth';

describe('AuthService', () => {
  let authService: AuthService;

  beforeEach(() => {
    authService = new AuthService();
  });

  describe('validateApiKey', () => {
    it('should validate correct API keys', () => {
      expect(authService.validateApiKey('test-key-1')).toBe(true);
      expect(authService.validateApiKey('test-key-2')).toBe(true);
      expect(authService.validateApiKey('admin-key')).toBe(true);
    });

    it('should reject invalid API keys', () => {
      expect(authService.validateApiKey('invalid-key')).toBe(false);
      expect(authService.validateApiKey('')).toBe(false);
      expect(authService.validateApiKey('test-key-3')).toBe(false);
    });

    it('should handle null/undefined keys', () => {
      expect(authService.validateApiKey(null as any)).toBe(false);
      expect(authService.validateApiKey(undefined as any)).toBe(false);
    });
  });

  describe('validateJWT', () => {
    const validPayload: Omit<JWTPayload, 'iat' | 'exp'> = {
      sub: 'user123',
      scope: ['robot:control', 'robot:read'],
    };

    it('should validate correct JWT tokens', () => {
      const token = jwt.sign(validPayload, 'test-secret-key', { expiresIn: '1h' });
      const decoded = authService.validateJWT(token);
      
      expect(decoded).toBeTruthy();
      expect(decoded!.sub).toBe('user123');
      expect(decoded!.scope).toEqual(['robot:control', 'robot:read']);
    });

    it('should reject expired tokens', () => {
      const token = jwt.sign(validPayload, 'test-secret-key', { expiresIn: '-1h' });
      const decoded = authService.validateJWT(token);
      
      expect(decoded).toBeNull();
    });

    it('should reject tokens with invalid signature', () => {
      const token = jwt.sign(validPayload, 'wrong-secret', { expiresIn: '1h' });
      const decoded = authService.validateJWT(token);
      
      expect(decoded).toBeNull();
    });

    it('should reject malformed tokens', () => {
      expect(authService.validateJWT('invalid-token')).toBeNull();
      expect(authService.validateJWT('')).toBeNull();
      expect(authService.validateJWT('not.a.jwt')).toBeNull();
    });

    it('should handle null/undefined tokens', () => {
      expect(authService.validateJWT(null as any)).toBeNull();
      expect(authService.validateJWT(undefined as any)).toBeNull();
    });
  });

  describe('generateJWT', () => {
    it('should generate valid JWT tokens', () => {
      const token = authService.generateJWT('user123', ['robot:control']);
      const decoded = jwt.verify(token, 'test-secret-key') as JWTPayload;
      
      expect(decoded.sub).toBe('user123');
      expect(decoded.scope).toEqual(['robot:control']);
      expect(decoded.iat).toBeTruthy();
      expect(decoded.exp).toBeTruthy();
    });

    it('should generate tokens with default empty scopes', () => {
      const token = authService.generateJWT('user123');
      const decoded = jwt.verify(token, 'test-secret-key') as JWTPayload;
      
      expect(decoded.sub).toBe('user123');
      expect(decoded.scope).toEqual([]);
    });

    it('should throw error when JWT_SECRET is not configured', () => {
      // Create a mock AuthService with empty JWT_SECRET
      const mockAuthService = new AuthService();
      // Override the config property
      (mockAuthService as any).config = { JWT_SECRET: '' };
      
      expect(() => mockAuthService.generateJWT('user123')).toThrow('JWT_SECRET not configured');
    });
  });

  describe('extractAuth', () => {
    it('should extract API key from Authorization header', () => {
      const req = {
        headers: { authorization: 'ApiKey test-key-1' },
        query: {},
      } as unknown as Request;

      const auth = authService.extractAuth(req);
      expect(auth.apiKey).toBe('test-key-1');
    });

    it('should extract API key from query parameter', () => {
      const req = {
        headers: {},
        query: { api_key: 'test-key-2' },
      } as unknown as Request;

      const auth = authService.extractAuth(req);
      expect(auth.apiKey).toBe('test-key-2');
    });

    it('should extract JWT token from Authorization header', () => {
      const token = jwt.sign({ sub: 'user123', scope: [] }, 'test-secret-key', { expiresIn: '1h' });
      const req = {
        headers: { authorization: `Bearer ${token}` },
        query: {},
      } as unknown as Request;

      const auth = authService.extractAuth(req);
      expect(auth.token).toBe(token);
      expect(auth.user).toBeTruthy();
      expect(auth.user!.id).toBe('user123');
    });

    it('should handle requests with no authentication', () => {
      const req = {
        headers: {},
        query: {},
      } as unknown as Request;

      const auth = authService.extractAuth(req);
      expect(auth.apiKey).toBeUndefined();
      expect(auth.token).toBeUndefined();
      expect(auth.user).toBeUndefined();
    });

    it('should prioritize Authorization header over query parameter for API key', () => {
      const req = {
        headers: { authorization: 'ApiKey header-key' },
        query: { api_key: 'query-key' },
      } as unknown as Request;

      const auth = authService.extractAuth(req);
      expect(auth.apiKey).toBe('header-key');
    });
  });

  describe('isAuthEnabled', () => {
    it('should return true when authentication is enabled', () => {
      expect(authService.isAuthEnabled()).toBe(true);
    });
  });
});

describe('requireAuth middleware', () => {
  let req: AuthenticatedRequest;
  let res: Response;
  let next: NextFunction;

  beforeEach(() => {
    req = {
      headers: {},
      query: {},
      ip: '127.0.0.1',
    } as AuthenticatedRequest;
    
    res = {
      status: jest.fn().mockReturnThis(),
      json: jest.fn(),
    } as any;
    
    next = jest.fn();
  });

  it('should allow requests with valid API key', () => {
    req.headers.authorization = 'ApiKey test-key-1';
    
    const middleware = requireAuth();
    middleware(req, res, next);
    
    expect(next).toHaveBeenCalled();
    expect(req.auth?.apiKey).toBe('test-key-1');
  });

  it('should allow requests with valid JWT token', () => {
    const token = jwt.sign({ sub: 'user123', scope: ['robot:control'] }, 'test-secret-key', { expiresIn: '1h' });
    req.headers.authorization = `Bearer ${token}`;
    
    const middleware = requireAuth();
    middleware(req, res, next);
    
    expect(next).toHaveBeenCalled();
    expect(req.auth?.user?.id).toBe('user123');
  });

  it('should reject requests without authentication', () => {
    const middleware = requireAuth();
    middleware(req, res, next);
    
    expect(res.status).toHaveBeenCalledWith(401);
    expect(res.json).toHaveBeenCalledWith({
      ok: false,
      error: 'Authentication required. Provide valid API key or JWT token.',
      timestamp: expect.any(String),
    });
    expect(next).not.toHaveBeenCalled();
  });

  it('should reject requests with invalid API key', () => {
    req.headers.authorization = 'ApiKey invalid-key';
    
    const middleware = requireAuth();
    middleware(req, res, next);
    
    expect(res.status).toHaveBeenCalledWith(401);
    expect(next).not.toHaveBeenCalled();
  });

  it('should reject requests with expired JWT token', () => {
    const token = jwt.sign({ sub: 'user123', scope: [] }, 'test-secret-key', { expiresIn: '-1h' });
    req.headers.authorization = `Bearer ${token}`;
    
    const middleware = requireAuth();
    middleware(req, res, next);
    
    expect(res.status).toHaveBeenCalledWith(401);
    expect(next).not.toHaveBeenCalled();
  });

  it('should check required scopes for JWT tokens', () => {
    const token = jwt.sign({ sub: 'user123', scope: ['robot:read'] }, 'test-secret-key', { expiresIn: '1h' });
    req.headers.authorization = `Bearer ${token}`;
    
    const middleware = requireAuth(['robot:control']);
    middleware(req, res, next);
    
    expect(res.status).toHaveBeenCalledWith(403);
    expect(res.json).toHaveBeenCalledWith({
      ok: false,
      error: 'Insufficient permissions',
      timestamp: expect.any(String),
    });
    expect(next).not.toHaveBeenCalled();
  });

  it('should allow wildcard scope', () => {
    const token = jwt.sign({ sub: 'user123', scope: ['*'] }, 'test-secret-key', { expiresIn: '1h' });
    req.headers.authorization = `Bearer ${token}`;
    
    const middleware = requireAuth(['robot:control']);
    middleware(req, res, next);
    
    expect(next).toHaveBeenCalled();
  });

  it('should skip authentication when disabled', () => {
    // Mock the authService.isAuthEnabled method to return false
    const { authService } = require('../auth');
    const originalIsAuthEnabled = authService.isAuthEnabled;
    authService.isAuthEnabled = jest.fn().mockReturnValue(false);
    
    const middleware = requireAuth();
    middleware(req, res, next);
    
    expect(next).toHaveBeenCalled();
    
    // Restore original method
    authService.isAuthEnabled = originalIsAuthEnabled;
  });
});

describe('addAuthContext middleware', () => {
  let req: AuthenticatedRequest;
  let res: Response;
  let next: NextFunction;

  beforeEach(() => {
    req = {
      headers: {},
      query: {},
    } as AuthenticatedRequest;
    
    res = {} as Response;
    next = jest.fn();
  });

  it('should add auth context to request when authentication is enabled', () => {
    req.headers.authorization = 'ApiKey test-key-1';
    
    addAuthContext(req, res, next);
    
    expect(req.auth).toBeTruthy();
    expect(req.auth!.apiKey).toBe('test-key-1');
    expect(next).toHaveBeenCalled();
  });

  it('should not add auth context when authentication is disabled', () => {
    // Mock the authService.isAuthEnabled method to return false
    const { authService } = require('../auth');
    const originalIsAuthEnabled = authService.isAuthEnabled;
    authService.isAuthEnabled = jest.fn().mockReturnValue(false);

    addAuthContext(req, res, next);
    
    expect(req.auth).toBeUndefined();
    expect(next).toHaveBeenCalled();
    
    // Restore original method
    authService.isAuthEnabled = originalIsAuthEnabled;
  });
});

describe('createRateLimit', () => {
  it('should create rate limit middleware with default configuration', () => {
    const rateLimitMiddleware = createRateLimit();
    expect(rateLimitMiddleware).toBeTruthy();
  });

  it('should create rate limit middleware with custom key generator', () => {
    const customKeyGenerator = (req: Request) => `custom:${req.ip}`;
    const rateLimitMiddleware = createRateLimit(customKeyGenerator);
    expect(rateLimitMiddleware).toBeTruthy();
  });
});