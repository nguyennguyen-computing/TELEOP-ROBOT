/**
 * Authentication middleware for API key and JWT token validation
 */

import { Request, Response, NextFunction } from 'express';
import jwt from 'jsonwebtoken';
import rateLimit from 'express-rate-limit';
import { AuthRequest, JWTPayload, AuthenticatedRequest } from '@web-teleop-robot/shared';
import { getEnvironmentConfig } from '../config/environment';

/**
 * Authentication service for handling API keys and JWT tokens
 */
export class AuthService {
  private config = getEnvironmentConfig();
  private apiKeys: string[];

  constructor() {
    this.apiKeys = this.config.API_KEYS ? this.config.API_KEYS.split(',').map(key => key.trim()) : [];
  }

  /**
   * Validates an API key
   * @param key API key to validate
   * @returns True if valid, false otherwise
   */
  validateApiKey(key: string): boolean {
    if (!key || this.apiKeys.length === 0) {
      return false;
    }
    return this.apiKeys.includes(key);
  }

  /**
   * Validates and decodes a JWT token
   * @param token JWT token to validate
   * @returns Decoded payload if valid, null otherwise
   */
  validateJWT(token: string): JWTPayload | null {
    if (!token || !this.config.JWT_SECRET) {
      return null;
    }

    try {
      const decoded = jwt.verify(token, this.config.JWT_SECRET) as JWTPayload;
      
      // Validate required fields
      if (!decoded.sub || !decoded.iat || !decoded.exp) {
        return null;
      }

      // Check if token is expired
      if (decoded.exp * 1000 < Date.now()) {
        return null;
      }

      return decoded;
    } catch (error) {
      return null;
    }
  }

  /**
   * Generates a JWT token for a user
   * @param userId User ID
   * @param scopes User scopes/permissions
   * @returns JWT token string
   */
  generateJWT(userId: string, scopes: string[] = []): string {
    if (!this.config.JWT_SECRET) {
      throw new Error('JWT_SECRET not configured');
    }

    const payload: Omit<JWTPayload, 'iat' | 'exp'> = {
      sub: userId,
      scope: scopes,
    };

    return jwt.sign(payload, this.config.JWT_SECRET, {
      expiresIn: this.config.JWT_EXPIRES_IN || '24h',
    } as jwt.SignOptions);
  }

  /**
   * Extracts authentication information from request
   * @param req Express request object
   * @returns Authentication information
   */
  extractAuth(req: Request): AuthRequest {
    const authHeader = req.headers.authorization;
    const apiKeyQuery = req.query.api_key as string;
    
    let apiKey: string | undefined;
    let token: string | undefined;

    // Extract API key from Authorization header or query parameter
    if (authHeader && authHeader.startsWith('ApiKey ')) {
      apiKey = authHeader.substring(7);
    } else if (apiKeyQuery) {
      apiKey = apiKeyQuery;
    }

    // Extract JWT token from Authorization header
    if (authHeader && authHeader.startsWith('Bearer ')) {
      token = authHeader.substring(7);
    }

    const auth: AuthRequest = {};

    if (apiKey) {
      auth.apiKey = apiKey;
    }

    if (token) {
      auth.token = token;
      const payload = this.validateJWT(token);
      if (payload) {
        auth.user = {
          id: payload.sub,
          scopes: payload.scope || [],
        };
      }
    }

    return auth;
  }

  /**
   * Checks if authentication is enabled
   * @returns True if authentication is enabled
   */
  isAuthEnabled(): boolean {
    return this.config.AUTH_ENABLED === true;
  }
}

// Singleton instance - lazy initialization
let _authService: AuthService | null = null;

function getAuthService(): AuthService {
  if (!_authService) {
    _authService = new AuthService();
  }
  return _authService;
}

/**
 * Authentication middleware that validates API keys or JWT tokens
 * @param scopes Optional array of required scopes
 * @returns Express middleware function
 */
export function requireAuth(scopes: string[] = []) {
  return (req: AuthenticatedRequest, res: Response, next: NextFunction) => {
    const authService = getAuthService();
    
    // Skip authentication if disabled
    if (!authService.isAuthEnabled()) {
      return next();
    }

    const auth = authService.extractAuth(req);
    
    // Check API key authentication
    if (auth.apiKey && authService.validateApiKey(auth.apiKey)) {
      req.auth = auth;
      return next();
    }

    // Check JWT authentication
    if (auth.token && auth.user) {
      // Check required scopes
      if (scopes.length > 0) {
        const hasRequiredScope = scopes.some(scope => 
          auth.user!.scopes.includes(scope) || auth.user!.scopes.includes('*')
        );
        
        if (!hasRequiredScope) {
          return res.status(403).json({
            ok: false,
            error: 'Insufficient permissions',
            timestamp: new Date().toISOString(),
          });
        }
      }

      req.auth = auth;
      return next();
    }

    // Authentication failed
    return res.status(401).json({
      ok: false,
      error: 'Authentication required. Provide valid API key or JWT token.',
      timestamp: new Date().toISOString(),
    });
  };
}

/**
 * Rate limiting middleware factory
 * @param keyGenerator Function to generate rate limit key from request
 * @returns Express rate limit middleware
 */
export function createRateLimit(keyGenerator?: (req: Request) => string) {
  return (req: Request, res: Response, next: NextFunction) => {
    const config = getEnvironmentConfig();
    
    const rateLimitMiddleware = rateLimit({
      windowMs: config.RATE_LIMIT_WINDOW_MS || 900000, // 15 minutes
      max: config.RATE_LIMIT_MAX_REQUESTS || 100,
      keyGenerator: keyGenerator || ((req: AuthenticatedRequest) => {
        // Use API key, user ID, or IP address for rate limiting
        if (req.auth?.apiKey) {
          return `api_key:${req.auth.apiKey}`;
        }
        if (req.auth?.user?.id) {
          return `user:${req.auth.user.id}`;
        }
        return req.ip || 'unknown';
      }),
      message: {
        ok: false,
        error: 'Too many requests. Please try again later.',
        timestamp: new Date().toISOString(),
      },
      standardHeaders: true,
      legacyHeaders: false,
    });
    
    return rateLimitMiddleware(req, res, next);
  };
}

/**
 * Middleware to add authentication context to request
 */
export function addAuthContext(req: AuthenticatedRequest, _res: Response, next: NextFunction) {
  const authService = getAuthService();
  if (authService.isAuthEnabled()) {
    req.auth = authService.extractAuth(req);
  }
  next();
}

export const authService = {
  generateJWT: (userId: string, scopes: string[] = []) => getAuthService().generateJWT(userId, scopes),
  validateJWT: (token: string) => getAuthService().validateJWT(token),
  validateApiKey: (key: string) => getAuthService().validateApiKey(key),
  extractAuth: (req: Request) => getAuthService().extractAuth(req),
  isAuthEnabled: () => getAuthService().isAuthEnabled(),
};