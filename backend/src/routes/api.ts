import { Router, Response } from 'express';
import { ApiResponse, VelocityCommand, VelocityResponse, VelocityLogEntry, AuthenticatedRequest } from '@web-teleop-robot/shared';
import { velocityProcessor } from '../services/velocityProcessor';
import { fleetClient } from '../services/fleetClient';
import { createDatabaseService } from '../database/models';
import { broadcastVelocityUpdate, getConnectionStats } from '../websocket/server';
import { requireAuth, createRateLimit, addAuthContext, authService } from '../middleware/auth';

/**
 * Request body interface for velocity endpoint
 */
interface VelocityRequest {
  vx: number;
  vy: number;
  levels: {
    up: number;
    down: number;
    left: number;
    right: number;
  };
}

/**
 * Query parameters interface for logs endpoint
 */
interface LogsQuery {
  limit?: string;
  skip?: string;
  source?: string;
  startDate?: string;
  endDate?: string;
  sortBy?: 'ts' | 'vx' | 'vy';
  sortOrder?: '1' | '-1';
}

/**
 * Creates and configures API routes
 * @returns Express router with API endpoints
 */
export function createApiRoutes(): Router {
  const router = Router();

  // Add authentication context to all requests
  router.use(addAuthContext);

  // Apply rate limiting to all API routes
  router.use(createRateLimit());

  // API health check
  router.get('/health', (_req, res) => {
    const response: ApiResponse = {
      ok: true,
      data: {
        service: 'api',
        timestamp: new Date().toISOString(),
        version: '1.0.0',
      },
      timestamp: new Date().toISOString(),
    };

    res.json(response);
  });

  router.post('/auth/token', requireAuth(), async (req: AuthenticatedRequest, res) => {
    try {
      const { userId, scopes } = req.body;

      // Validate request body
      if (!userId || typeof userId !== 'string') {
        return res.status(400).json({
          ok: false,
          error: 'Invalid request body. Expected: { userId: string, scopes?: string[] }',
          timestamp: new Date().toISOString(),
        });
      }

      const userScopes = Array.isArray(scopes) ? scopes : ['robot:read', 'robot:control'];

      // Generate JWT token
      const token = authService.generateJWT(userId, userScopes);

      const response: ApiResponse = {
        ok: true,
        data: {
          token,
          userId,
          scopes: userScopes,
          expiresIn: '24h',
        },
        timestamp: new Date().toISOString(),
      };

      return res.status(200).json(response);

    } catch (error) {
      console.error('Error generating JWT token:', error);
      
      const response: ApiResponse = {
        ok: false,
        error: error instanceof Error ? error.message : 'Internal server error',
        timestamp: new Date().toISOString(),
      };

      return res.status(500).json(response);
    }
  });

  router.get('/auth/verify', requireAuth(), (req: AuthenticatedRequest, res) => {
    const response: ApiResponse = {
      ok: true,
      data: {
        authenticated: true,
        auth: req.auth,
      },
      timestamp: new Date().toISOString(),
    };

    res.json(response);
  });

  router.get('/websocket/stats', requireAuth(['robot:read']), (_req, res) => {
    try {
      const stats = getConnectionStats();
      
      const response: ApiResponse = {
        ok: true,
        data: stats,
        timestamp: new Date().toISOString(),
      };

      return res.status(200).json(response);
    } catch (error) {
      console.error('Error retrieving WebSocket stats:', error);
      
      const response: ApiResponse = {
        ok: false,
        error: error instanceof Error ? error.message : 'Internal server error',
        timestamp: new Date().toISOString(),
      };

      return res.status(500).json(response);
    }
  });

  router.get('/fleet/stats', requireAuth(['robot:read']), (_req, res) => {
    try {
      const stats = fleetClient.getStats();
      
      const response: ApiResponse = {
        ok: true,
        data: stats,
        timestamp: new Date().toISOString(),
      };

      return res.status(200).json(response);
    } catch (error) {
      console.error('Error retrieving fleet client stats:', error);
      
      const response: ApiResponse = {
        ok: false,
        error: error instanceof Error ? error.message : 'Internal server error',
        timestamp: new Date().toISOString(),
      };

      return res.status(500).json(response);
    }
  });

  router.get('/fleet/health', requireAuth(['robot:read']), async (_req, res) => {
    try {
      const healthCheck = await fleetClient.healthCheck();
      
      const response: ApiResponse = {
        ok: healthCheck.healthy,
        data: healthCheck,
        timestamp: new Date().toISOString(),
      };

      return res.status(healthCheck.healthy ? 200 : 503).json(response);
    } catch (error) {
      console.error('Error checking fleet server health:', error);
      
      const response: ApiResponse = {
        ok: false,
        error: error instanceof Error ? error.message : 'Internal server error',
        timestamp: new Date().toISOString(),
      };

      return res.status(500).json(response);
    }
  });

  router.post('/vel', requireAuth(['robot:control']), async (req: AuthenticatedRequest, res: Response<VelocityResponse>) => {
    try {
      const { vx, vy, levels } = req.body as VelocityRequest;

      // Validate request body structure
      if (typeof vx !== 'number' || typeof vy !== 'number' || !levels) {
        return res.status(400).json({
          ok: false,
          error: 'Invalid request body. Expected: { vx: number, vy: number, levels: { up, down, left, right } }',
          timestamp: new Date().toISOString(),
        });
      }

      // Create velocity command
      const command: VelocityCommand = {
        vx,
        vy,
        levels,
        timestamp: new Date(),
        source: 'web'
      };

      // Validate the velocity command
      const validation = velocityProcessor.validateCommand(command);
      if (!validation.isValid) {
        return res.status(400).json({
          ok: false,
          error: `Validation failed: ${validation.errors.join(', ')}`,
          timestamp: new Date().toISOString(),
        });
      }

      // Log the command to database
      try {
        const dbService = await createDatabaseService();
        const velocityLogModel = dbService.getVelocityLogModel();
        await velocityLogModel.insertLog(vx, vy, levels, 'web');
      } catch (dbError) {
        console.error('Failed to log velocity command to database:', dbError);
        // Continue processing even if logging fails
      }

      // Broadcast velocity update to WebSocket clients
      try {
        broadcastVelocityUpdate(command);
      } catch (wsError) {
        console.error('Failed to broadcast velocity update via WebSocket:', wsError);
        // Continue processing even if WebSocket broadcast fails
      }

      let fleetResponse;
      let fleetError: string | undefined;
      
      try {
        fleetResponse = await fleetClient.sendVelocityCommand(command);
        console.log('Successfully forwarded command to fleet server:', fleetResponse);
      } catch (fleetErr) {
        fleetError = fleetErr instanceof Error ? fleetErr.message : 'Fleet server communication failed';
        console.error('Failed to forward command to fleet server:', fleetError);
        
        // Don't fail the entire request if fleet forwarding fails
        // The command is still logged and broadcast to WebSocket clients
      }

      // Return success response
      const response: VelocityResponse = {
        ok: true,
        published: fleetResponse?.published || false,
        data: {
          vx: command.vx,
          vy: command.vy,
          levels: command.levels,
          timestamp: command.timestamp?.toISOString(),
          fleetStatus: fleetError ? 'error' : 'success',
          fleetError: fleetError
        },
        timestamp: new Date().toISOString(),
      };

      return res.status(200).json(response);

    } catch (error) {
      console.error('Error processing velocity command:', error);
      
      const response: VelocityResponse = {
        ok: false,
        error: error instanceof Error ? error.message : 'Internal server error',
        timestamp: new Date().toISOString(),
      };

      return res.status(500).json(response);
    }
  });

  router.get('/logs', requireAuth(['robot:read']), async (req: AuthenticatedRequest, res) => {
    try {
      // Parse query parameters with defaults
      const query = req.query as LogsQuery;
      const limit = Math.min(parseInt(query.limit || '100'), 1000); // Cap at 1000
      const skip = Math.max(parseInt(query.skip || '0'), 0);
      const source = query.source;
      const sortBy = query.sortBy || 'ts';
      const sortOrder = query.sortOrder === '1' ? 1 : -1;

      // Parse date filters
      let startDate: Date | undefined;
      let endDate: Date | undefined;

      if (query.startDate) {
        startDate = new Date(query.startDate);
        if (isNaN(startDate.getTime())) {
          return res.status(400).json({
            ok: false,
            error: 'Invalid startDate format. Use ISO 8601 format (e.g., 2023-12-01T00:00:00Z)',
            timestamp: new Date().toISOString(),
          });
        }
      }

      if (query.endDate) {
        endDate = new Date(query.endDate);
        if (isNaN(endDate.getTime())) {
          return res.status(400).json({
            ok: false,
            error: 'Invalid endDate format. Use ISO 8601 format (e.g., 2023-12-01T23:59:59Z)',
            timestamp: new Date().toISOString(),
          });
        }
      }

      // Validate sortBy parameter
      if (!['ts', 'vx', 'vy'].includes(sortBy)) {
        return res.status(400).json({
          ok: false,
          error: 'Invalid sortBy parameter. Must be one of: ts, vx, vy',
          timestamp: new Date().toISOString(),
        });
      }

      // Get database service and query logs
      const dbService = await createDatabaseService();
      const velocityLogModel = dbService.getVelocityLogModel();

      const queryOptions: any = {
        limit,
        skip,
        sortBy: sortBy as 'ts' | 'vx' | 'vy',
        sortOrder: sortOrder as 1 | -1
      };

      if (source) queryOptions.source = source;
      if (startDate) queryOptions.startDate = startDate;
      if (endDate) queryOptions.endDate = endDate;

      const countOptions: any = {};
      if (source) countOptions.source = source;
      if (startDate) countOptions.startDate = startDate;
      if (endDate) countOptions.endDate = endDate;

      // Execute queries in parallel
      const [logs, total] = await Promise.all([
        velocityLogModel.getLogs(queryOptions),
        velocityLogModel.getLogsCount(countOptions)
      ]);

      // Calculate pagination info
      const hasMore = skip + logs.length < total;
      const nextSkip = hasMore ? skip + limit : null;
      const prevSkip = skip > 0 ? Math.max(0, skip - limit) : null;

      const response: ApiResponse<{ logs: VelocityLogEntry[]; total: number; pagination: any }> = {
        ok: true,
        data: {
          logs,
          total,
          pagination: {
            limit,
            skip,
            count: logs.length,
            hasMore,
            nextSkip,
            prevSkip,
            totalPages: Math.ceil(total / limit),
            currentPage: Math.floor(skip / limit) + 1
          }
        },
        timestamp: new Date().toISOString(),
      };

      return res.status(200).json(response);

    } catch (error) {
      console.error('Error retrieving velocity logs:', error);
      
      const response: ApiResponse = {
        ok: false,
        error: error instanceof Error ? error.message : 'Internal server error',
        timestamp: new Date().toISOString(),
      };

      return res.status(500).json(response);
    }
  });

  return router;
}