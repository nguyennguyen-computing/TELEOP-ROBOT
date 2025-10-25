import { WebSocketServer, WebSocket } from 'ws';
import { IncomingMessage } from 'http';
import { URL } from 'url';
import {
  WebSocketMessage,
  ConnectionStatusMessage,
  SetVelMessage,
  VelocityCommand
} from '@web-teleop-robot/shared';
import { logEvent } from '../middleware/requestLogger';
import { velocityProcessor } from '../services/velocityProcessor';
import { createDatabaseService } from '../database/models';
import { FleetVelocityResponse, FleetClient } from '../services';
import { authService } from '../middleware/auth';

// Store active WebSocket connections with metadata
interface WebSocketConnection {
  ws: WebSocket;
  id: string;
  clientIp: string;
  connectedAt: Date;
  lastPing: Date;
  isAlive: boolean;
  authenticated: boolean;
  userId?: string | undefined;
  scopes?: string[] | undefined;
}

const activeConnections = new Map<WebSocket, WebSocketConnection>();

// Heartbeat configuration
const HEARTBEAT_INTERVAL = 30000; // 30 seconds

/**
 * Authenticates WebSocket connection using query parameters
 * @param req Incoming HTTP request
 * @returns Authentication result
 */
function authenticateWebSocketConnection(req: IncomingMessage): {
  authenticated: boolean;
  userId?: string;
  scopes?: string[];
  error?: string;
} {
  // Skip authentication if disabled
  if (!authService.isAuthEnabled()) {
    return { authenticated: true };
  }

  try {
    const url = new URL(req.url || '', `http://${req.headers.host}`);
    const token = url.searchParams.get('token');
    const apiKey = url.searchParams.get('apiKey');

    // Try JWT token first
    if (token) {
      const payload = authService.validateJWT(token);
      if (payload) {
        return {
          authenticated: true,
          userId: payload.sub,
          scopes: payload.scope || []
        };
      }
      return { authenticated: false, error: 'Invalid JWT token' };
    }

    // Try API key
    if (apiKey) {
      if (authService.validateApiKey(apiKey)) {
        return {
          authenticated: true,
          userId: 'api-key-user', // API key doesn't have specific user
          scopes: ['robot:read', 'robot:control'] // Default scopes for API key
        };
      }
      return { authenticated: false, error: 'Invalid API key' };
    }

    return { authenticated: false, error: 'No authentication provided' };
  } catch (error) {
    return {
      authenticated: false,
      error: error instanceof Error ? error.message : 'Authentication error'
    };
  }
}

/**
 * Sets up WebSocket server with connection handling and heartbeat
 * @param wss WebSocket server instance
 */
export function setupWebSocket(wss: WebSocketServer): void {
  wss.on('connection', (ws: WebSocket, req) => {
    const clientIp = req.socket.remoteAddress || 'unknown';
    const connectionId = generateConnectionId();
    const now = new Date();

    // Authenticate connection
    const authResult = authenticateWebSocketConnection(req);

    if (!authResult.authenticated) {
      logEvent('websocket_auth_failed', {
        connectionId,
        clientIp,
        error: authResult.error,
      }, 'warn');

      // Send authentication error and close connection
      const errorMessage: WebSocketMessage = {
        type: 'error',
        data: {
          message: 'Authentication required',
          error: authResult.error
        },
        timestamp: new Date().toISOString(),
      };

      ws.send(JSON.stringify(errorMessage));
      ws.close(1008, 'Authentication required'); // Policy violation
      return;
    }

    // Create connection metadata
    const connection: WebSocketConnection = {
      ws,
      id: connectionId,
      clientIp,
      connectedAt: now,
      lastPing: now,
      isAlive: true,
      authenticated: authResult.authenticated,
      userId: authResult.userId,
      scopes: authResult.scopes
    };

    // Add to active connections
    activeConnections.set(ws, connection);

    logEvent('websocket_connection', {
      connectionId,
      clientIp,
      totalConnections: activeConnections.size,
    });

    // Send connection status to all clients
    broadcastConnectionStatus();

    // Set up heartbeat for this connection
    ws.isAlive = true;

    // Handle pong responses (heartbeat)
    ws.on('pong', () => {
      const conn = activeConnections.get(ws);
      if (conn) {
        conn.isAlive = true;
        conn.lastPing = new Date();
        ws.isAlive = true;
      }
    });

    // Handle incoming messages
    ws.on('message', (data: Buffer) => {
      try {
        const message: WebSocketMessage = JSON.parse(data.toString());

        // Reset deadman switch timeout on any message activity
        resetDeadmanTimeout(connectionId, clientIp);

        handleWebSocketMessage(ws, message, connection);
      } catch (error) {
        logEvent('websocket_message_error', {
          error: error instanceof Error ? error.message : 'Unknown error',
          connectionId,
          clientIp,
        }, 'error');

        // Send error response
        const errorMessage: WebSocketMessage = {
          type: 'error',
          data: { message: 'Invalid message format' },
          timestamp: new Date().toISOString(),
        };

        sendMessage(ws, errorMessage);
      }
    });

    // Handle connection close
    ws.on('close', async (code: number, reason: Buffer) => {
      activeConnections.delete(ws);

      // Clear deadman switch timeout
      clearDeadmanTimeout(connectionId);

      logEvent('websocket_disconnection', {
        connectionId,
        clientIp,
        code,
        reason: reason.toString(),
        totalConnections: activeConnections.size,
      });

      // DEADMAN SWITCH: Send stop command when client disconnects
      try {
        await sendDeadmanStopCommand(connectionId, clientIp);
      } catch (error) {
        logEvent('deadman_switch_error', {
          connectionId,
          clientIp,
          error: error instanceof Error ? error.message : 'Unknown error'
        }, 'error');
      }

      // Broadcast updated connection status
      broadcastConnectionStatus();
    });

    // Handle connection errors
    ws.on('error', async (error: Error) => {
      logEvent('websocket_error', {
        error: error.message,
        connectionId,
        clientIp,
      }, 'error');

      activeConnections.delete(ws);

      // DEADMAN SWITCH: Send stop command on connection error
      try {
        await sendDeadmanStopCommand(connectionId, clientIp);
      } catch (deadmanError) {
        logEvent('deadman_switch_error', {
          connectionId,
          clientIp,
          originalError: error.message,
          deadmanError: deadmanError instanceof Error ? deadmanError.message : 'Unknown error'
        }, 'error');
      }

      broadcastConnectionStatus();
    });

    // Setup deadman switch timeout for new connection
    resetDeadmanTimeout(connectionId, clientIp);

    // Send initial connection confirmation
    const welcomeMessage: ConnectionStatusMessage = {
      type: 'connection_status',
      data: {
        connected: true,
        clientCount: activeConnections.size,
        connectionId,
        message: 'Connected to Web Teleop Robot backend',
      },
      timestamp: new Date().toISOString(),
    };

    sendMessage(ws, welcomeMessage);
  });

  // Handle server errors
  wss.on('error', (error: Error) => {
    logEvent('websocket_server_error', {
      error: error.message,
    }, 'error');
  });

  // Set up heartbeat interval for connection monitoring
  const heartbeatInterval = setInterval(() => {
    performHeartbeatCheck();
  }, HEARTBEAT_INTERVAL);

  // Clean up interval on server close
  wss.on('close', () => {
    clearInterval(heartbeatInterval);
  });

  logEvent('websocket_server_started', {
    message: 'WebSocket server is ready for connections',
    heartbeatInterval: HEARTBEAT_INTERVAL,
  });
}

/**
 * Handles incoming WebSocket messages
 * @param ws WebSocket connection
 * @param message Parsed WebSocket message
 * @param connection Connection metadata
 */
async function handleWebSocketMessage(
  ws: WebSocket,
  message: WebSocketMessage,
  connection: WebSocketConnection
): Promise<void> {
  logEvent('websocket_message_received', {
    type: message.type,
    connectionId: connection.id,
    timestamp: message.timestamp,
  });

  switch (message.type) {
    case 'set_vel':
      await handleSetVelMessage(ws, message as SetVelMessage, connection);
      break;

    case 'connection_status':
      // Client requesting connection status
      sendConnectionStatus(ws, connection);
      break;

    case 'ping':
      // Handle ping from client, send pong back
      const originalTimestamp = message.data?.timestamp || Date.now()
      const pongMessage: WebSocketMessage = {
        type: 'pong',
        data: {
          timestamp: originalTimestamp, // Return original timestamp for latency calculation
          serverTime: Date.now() // Add server time for debugging
        },
        timestamp: new Date().toISOString(),
      };
      sendMessage(ws, pongMessage);

      // Update connection heartbeat
      connection.lastPing = new Date();
      connection.isAlive = true;

      logEvent('websocket_ping_received', {
        connectionId: connection.id,
        clientTimestamp: originalTimestamp,
        serverTime: Date.now()
      });
      break;

    case 'pong':
      // Handle pong from client (response to our ping)
      connection.lastPing = new Date();
      connection.isAlive = true;
      break;

    default:
      logEvent('websocket_unknown_message', {
        type: message.type,
        connectionId: connection.id,
      }, 'warn');

      const errorResponse: WebSocketMessage = {
        type: 'error',
        data: { message: `Unknown message type: ${message.type}` },
        timestamp: new Date().toISOString(),
      };

      sendMessage(ws, errorResponse);
  }
}

async function handleSetVelMessage(
  ws: WebSocket,
  message: SetVelMessage,
  connection: WebSocketConnection
): Promise<void> {
  try {
    // Check authorization for robot control
    if (!connection.authenticated || !connection.scopes?.includes('robot:control')) {
      const errorResponse: WebSocketMessage = {
        type: 'error',
        data: {
          message: 'Insufficient permissions. robot:control scope required.',
          code: 'INSUFFICIENT_PERMISSIONS'
        },
        timestamp: new Date().toISOString(),
      };

      sendMessage(ws, errorResponse);

      logEvent('websocket_set_vel_unauthorized', {
        connectionId: connection.id,
        userId: connection.userId,
        scopes: connection.scopes,
      }, 'warn');

      return;
    }

    const velocityCommand: VelocityCommand = {
      ...message.data,
      timestamp: new Date(),
      source: 'websocket'
    };

    logEvent('websocket_set_vel_received', {
      connectionId: connection.id,
      userId: connection.userId,
      vx: velocityCommand.vx,
      vy: velocityCommand.vy,
      levels: velocityCommand.levels,
    });

    // Validate the velocity command
    const validation = velocityProcessor.validateCommand(velocityCommand);
    if (!validation.isValid) {
      const errorResponse: WebSocketMessage = {
        type: 'error',
        data: {
          message: `Velocity validation failed: ${validation.errors.join(', ')}`,
          errors: validation.errors
        },
        timestamp: new Date().toISOString(),
      };

      sendMessage(ws, errorResponse);

      logEvent('websocket_set_vel_validation_failed', {
        connectionId: connection.id,
        errors: validation.errors,
      }, 'warn');

      return;
    }

    // Log the command to database
    try {
      const dbService = await createDatabaseService();
      const velocityLogModel = dbService.getVelocityLogModel();
      await velocityLogModel.insertLog(
        parseFloat(velocityCommand.vx.toFixed(1)),
        parseFloat(velocityCommand.vy.toFixed(1)),
        velocityCommand.levels,
        'websocket'
      );
    } catch (dbError) {
      logEvent('websocket_database_log_error', {
        connectionId: connection.id,
        error: dbError instanceof Error ? dbError.message : 'Unknown error',
      }, 'error');
      // Continue processing even if logging fails
    }

    let fleetResponse: FleetVelocityResponse | undefined;
    let fleetError: string | undefined;

    try {
      const { fleetClient } = await import('../services/fleetClient');
      fleetResponse = await fleetClient.sendVelocityCommand(velocityCommand);
      console.log('Successfully forwarded WebSocket command to fleet server:', fleetResponse);

      logEvent('websocket_fleet_forward_success', {
        connectionId: connection.id,
        vx: velocityCommand.vx,
        vy: velocityCommand.vy,
        fleetResponse: fleetResponse
      });
    } catch (fleetErr) {
      fleetError = fleetErr instanceof Error ? fleetErr.message : 'Fleet server communication failed';
      console.error('Failed to forward WebSocket command to fleet server:', fleetError);

      logEvent('websocket_fleet_forward_error', {
        connectionId: connection.id,
        vx: velocityCommand.vx,
        vy: velocityCommand.vy,
        error: fleetError
      }, 'error');

      // Don't fail the entire WebSocket request if fleet forwarding fails
      // The command is still logged and broadcast to WebSocket clients
    }

    // Send success response to the sender (after fleet forwarding attempt)
    const successResponse: WebSocketMessage = {
      type: 'set_vel',
      data: {
        success: true,
        published: fleetResponse?.published || false,
        vx: velocityCommand.vx,
        vy: velocityCommand.vy,
        levels: velocityCommand.levels,
        timestamp: velocityCommand.timestamp?.toISOString(),
        fleetStatus: fleetError ? 'error' : 'success',
        fleetError: fleetError
      },
      timestamp: new Date().toISOString(),
    };

    sendMessage(ws, successResponse);

    // Broadcast velocity update to all other connected clients
    const velocityUpdate: WebSocketMessage = {
      type: 'velocity_update',
      data: {
        vx: velocityCommand.vx,
        vy: velocityCommand.vy,
        levels: velocityCommand.levels,
        source: connection.id
      },
      timestamp: new Date().toISOString(),
    };

    broadcastMessage(velocityUpdate, ws); // Exclude sender

    logEvent('websocket_set_vel_processed', {
      connectionId: connection.id,
      vx: velocityCommand.vx,
      vy: velocityCommand.vy,
      broadcastToClients: activeConnections.size - 1,
      fleetStatus: fleetError ? 'error' : 'success',
      fleetError: fleetError
    });

  } catch (error) {
    logEvent('websocket_set_vel_error', {
      connectionId: connection.id,
      error: error instanceof Error ? error.message : 'Unknown error',
    }, 'error');

    const errorResponse: WebSocketMessage = {
      type: 'error',
      data: { message: 'Failed to process velocity command' },
      timestamp: new Date().toISOString(),
    };

    sendMessage(ws, errorResponse);
  }
}

/**
 * Sends connection status to a specific client
 */
function sendConnectionStatus(ws: WebSocket, connection: WebSocketConnection): void {
  const statusMessage: ConnectionStatusMessage = {
    type: 'connection_status',
    data: {
      connected: true,
      clientCount: activeConnections.size,
      connectionId: connection.id,
      connectedAt: connection.connectedAt.toISOString(),
      lastPing: connection.lastPing.toISOString(),
    },
    timestamp: new Date().toISOString(),
  };

  sendMessage(ws, statusMessage);
}

/**
 * Broadcasts connection status to all connected clients
 */
function broadcastConnectionStatus(): void {
  const statusMessage: ConnectionStatusMessage = {
    type: 'connection_status',
    data: {
      connected: true,
      clientCount: activeConnections.size,
    },
    timestamp: new Date().toISOString(),
  };

  broadcastMessage(statusMessage);

  logEvent('websocket_status_broadcast', {
    clientCount: activeConnections.size,
  });
}

/**
 * Safely sends a message to a WebSocket connection
 * @param ws WebSocket connection
 * @param message Message to send
 */
function sendMessage(ws: WebSocket, message: WebSocketMessage): void {
  if (ws.readyState === WebSocket.OPEN) {
    try {
      ws.send(JSON.stringify(message));
    } catch (error) {
      logEvent('websocket_send_error', {
        error: error instanceof Error ? error.message : 'Unknown error',
        messageType: message.type,
      }, 'error');
    }
  }
}

/**
 * Broadcasts a message to all connected clients
 * @param message Message to broadcast
 * @param excludeWs Optional WebSocket to exclude from broadcast
 */
export function broadcastMessage(message: WebSocketMessage, excludeWs?: WebSocket): void {
  const messageString = JSON.stringify(message);
  let sentCount = 0;

  activeConnections.forEach((connection, ws) => {
    if (ws !== excludeWs && ws.readyState === WebSocket.OPEN) {
      try {
        ws.send(messageString);
        sentCount++;
      } catch (error) {
        logEvent('websocket_broadcast_error', {
          error: error instanceof Error ? error.message : 'Unknown error',
          connectionId: connection.id,
          messageType: message.type,
        }, 'error');
      }
    }
  });

  logEvent('websocket_message_broadcast', {
    type: message.type,
    totalClients: activeConnections.size,
    sentToClients: sentCount,
  });
}

function performHeartbeatCheck(): void {
  const now = new Date();
  const deadConnections: WebSocket[] = [];

  activeConnections.forEach((connection, ws) => {
    // Check if connection is stale (no activity within extended timeout)
    const timeSinceLastPing = now.getTime() - connection.lastPing.getTime();

    // Use longer timeout for server-side check (60 seconds)
    // This gives client time to send its own pings
    if (timeSinceLastPing > 60000) {
      logEvent('websocket_connection_stale', {
        connectionId: connection.id,
        timeSinceLastPing,
        lastPing: connection.lastPing.toISOString()
      }, 'warn');

      deadConnections.push(ws);
      return;
    }

    // Only send server ping if we haven't heard from client in a while
    // and we haven't marked it as not alive yet
    if (timeSinceLastPing > 45000 && connection.isAlive) {
      // Mark as not alive and send ping
      connection.isAlive = false;
      ws.isAlive = false;

      try {
        // Send ping message instead of WebSocket ping frame
        const pingMessage: WebSocketMessage = {
          type: 'ping',
          data: { timestamp: Date.now() },
          timestamp: new Date().toISOString(),
        };
        sendMessage(ws, pingMessage);

        logEvent('websocket_server_ping_sent', {
          connectionId: connection.id,
          timeSinceLastPing
        });
      } catch (error) {
        logEvent('websocket_ping_error', {
          connectionId: connection.id,
          error: error instanceof Error ? error.message : 'Unknown error',
        }, 'error');
        deadConnections.push(ws);
      }
    }
  });

  // Clean up dead connections
  deadConnections.forEach((ws) => {
    const connection = activeConnections.get(ws);
    if (connection) {
      logEvent('websocket_heartbeat_timeout', {
        connectionId: connection.id,
        clientIp: connection.clientIp,
        connectedDuration: now.getTime() - connection.connectedAt.getTime(),
        reason: 'No activity for 60+ seconds'
      });
    }

    ws.terminate();
    activeConnections.delete(ws);
  });

  if (deadConnections.length > 0) {
    broadcastConnectionStatus();
  }

  logEvent('websocket_heartbeat_check', {
    totalConnections: activeConnections.size,
    deadConnections: deadConnections.length,
    activeConnections: activeConnections.size - deadConnections.length,
  });
}

/**
 * Generates a unique connection ID
 */
function generateConnectionId(): string {
  return `conn_${Date.now()}_${Math.random().toString(36).substring(2, 11)}`;
}

/**
 * Gets the current number of active WebSocket connections
 * @returns Number of active connections
 */
export function getActiveConnectionCount(): number {
  return activeConnections.size;
}

/**
 * Gets connection statistics
 * @returns Connection statistics object
 */
export function getConnectionStats(): {
  totalConnections: number;
  connections: Array<{
    id: string;
    clientIp: string;
    connectedAt: string;
    lastPing: string;
    isAlive: boolean;
    connectedDuration: number;
  }>;
} {
  const now = new Date();
  const connections = Array.from(activeConnections.values()).map(conn => ({
    id: conn.id,
    clientIp: conn.clientIp,
    connectedAt: conn.connectedAt.toISOString(),
    lastPing: conn.lastPing.toISOString(),
    isAlive: conn.isAlive,
    connectedDuration: now.getTime() - conn.connectedAt.getTime(),
  }));

  return {
    totalConnections: activeConnections.size,
    connections,
  };
}

/**
 * Sends a velocity update to all connected clients
 * Used by REST API to notify WebSocket clients of velocity changes
 */
export function broadcastVelocityUpdate(velocityCommand: VelocityCommand): void {
  const velocityUpdate: WebSocketMessage = {
    type: 'velocity_update',
    data: {
      vx: velocityCommand.vx,
      vy: velocityCommand.vy,
      levels: velocityCommand.levels,
      source: 'api',
      timestamp: velocityCommand.timestamp?.toISOString()
    },
    timestamp: new Date().toISOString(),
  };

  broadcastMessage(velocityUpdate);
}
/**
 * DEADMAN SWITCH: Sends stop command when client disconnects
 * This is a critical safety feature that ensures robot stops when connection is lost
 */
async function sendDeadmanStopCommand(connectionId: string, clientIp: string): Promise<void> {
  const stopCommand: VelocityCommand = {
    vx: 0,
    vy: 0,
    levels: { up: 0, down: 0, left: 0, right: 0 },
    timestamp: new Date()
  };

  logEvent('deadman_switch_activated', {
    connectionId,
    clientIp,
    command: stopCommand,
    reason: 'Client disconnected - sending emergency stop'
  }, 'warn');

  try {
    // Send stop command to fleet server
    const fleetClient = new FleetClient();
    const fleetResponse = await fleetClient.sendVelocityCommand(stopCommand);

    // Log to database
    const dbService = await createDatabaseService();
    const velocityLogModel = dbService.getVelocityLogModel();
    await velocityLogModel.insertLog(
      parseFloat(stopCommand.vx.toFixed(1)),
      parseFloat(stopCommand.vy.toFixed(1)),
      stopCommand.levels,
      'deadman_switch'
    );

    // Broadcast stop command to remaining connected clients
    const stopMessage: WebSocketMessage = {
      type: 'velocity_update',
      data: {
        vx: 0,
        vy: 0,
        levels: { up: 0, down: 0, left: 0, right: 0 },
        source: 'deadman_switch',
        timestamp: new Date().toISOString()
      },
      timestamp: new Date().toISOString(),
    };

    broadcastMessage(stopMessage);

    logEvent('deadman_switch_success', {
      connectionId,
      clientIp,
      fleetResponse: fleetResponse ? 'success' : 'failed',
      broadcastToClients: activeConnections.size
    });

  } catch (error) {
    logEvent('deadman_switch_failed', {
      connectionId,
      clientIp,
      error: error instanceof Error ? error.message : 'Unknown error',
      command: stopCommand
    }, 'error');

    // Even if fleet communication fails, still broadcast stop to other clients
    const emergencyStopMessage: WebSocketMessage = {
      type: 'velocity_update',
      data: {
        vx: 0,
        vy: 0,
        levels: { up: 0, down: 0, left: 0, right: 0 },
        source: 'deadman_switch_emergency',
        timestamp: new Date().toISOString()
      },
      timestamp: new Date().toISOString(),
    };

    broadcastMessage(emergencyStopMessage);
    throw error; // Re-throw to be caught by caller
  }
}
/**
 * Connection timeout tracking for deadman switch
 */
const connectionTimeouts = new Map<string, NodeJS.Timeout>();

/**
 * Sets up deadman switch timeout for a connection
 */
function setupDeadmanTimeout(connectionId: string, clientIp: string): void {
  const timeoutMs = parseInt(process.env.DEADMAN_SWITCH_TIMEOUT_MS || '5000', 10);

  // Clear existing timeout
  const existingTimeout = connectionTimeouts.get(connectionId);
  if (existingTimeout) {
    clearTimeout(existingTimeout);
  }

  // Set new timeout
  const timeout = setTimeout(async () => {
    logEvent('deadman_switch_timeout', {
      connectionId,
      clientIp,
      timeoutMs,
      reason: 'No heartbeat received within timeout period'
    }, 'warn');

    try {
      await sendDeadmanStopCommand(connectionId, clientIp);
    } catch (error) {
      logEvent('deadman_switch_timeout_error', {
        connectionId,
        clientIp,
        error: error instanceof Error ? error.message : 'Unknown error'
      }, 'error');
    }

    connectionTimeouts.delete(connectionId);
  }, timeoutMs);

  connectionTimeouts.set(connectionId, timeout);
}

/**
 * Resets deadman switch timeout (call on heartbeat/activity)
 */
function resetDeadmanTimeout(connectionId: string, clientIp: string): void {
  if (process.env.DEADMAN_SWITCH_ENABLED === 'true') {
    setupDeadmanTimeout(connectionId, clientIp);
  }
}

/**
 * Clears deadman switch timeout for a connection
 */
function clearDeadmanTimeout(connectionId: string): void {
  const timeout = connectionTimeouts.get(connectionId);
  if (timeout) {
    clearTimeout(timeout);
    connectionTimeouts.delete(connectionId);
  }
}