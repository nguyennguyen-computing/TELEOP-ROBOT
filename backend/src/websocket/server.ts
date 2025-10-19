import { WebSocketServer, WebSocket } from 'ws';
import { 
  WebSocketMessage, 
  ConnectionStatusMessage, 
  SetVelMessage, 
  VelocityCommand 
} from '@web-teleop-robot/shared';
import { logEvent } from '../middleware/requestLogger';
import { velocityProcessor } from '../services/velocityProcessor';
import { createDatabaseService } from '../database/models';

// Store active WebSocket connections with metadata
interface WebSocketConnection {
  ws: WebSocket;
  id: string;
  clientIp: string;
  connectedAt: Date;
  lastPing: Date;
  isAlive: boolean;
}

const activeConnections = new Map<WebSocket, WebSocketConnection>();

// Heartbeat configuration
const HEARTBEAT_INTERVAL = 30000; // 30 seconds
const HEARTBEAT_TIMEOUT = 10000;  // 10 seconds timeout for pong response

/**
 * Sets up WebSocket server with connection handling and heartbeat
 * @param wss WebSocket server instance
 */
export function setupWebSocket(wss: WebSocketServer): void {
  wss.on('connection', (ws: WebSocket, req) => {
    const clientIp = req.socket.remoteAddress || 'unknown';
    const connectionId = generateConnectionId();
    const now = new Date();
    
    // Create connection metadata
    const connection: WebSocketConnection = {
      ws,
      id: connectionId,
      clientIp,
      connectedAt: now,
      lastPing: now,
      isAlive: true
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
    ws.on('close', (code: number, reason: Buffer) => {
      activeConnections.delete(ws);
      
      logEvent('websocket_disconnection', {
        connectionId,
        clientIp,
        code,
        reason: reason.toString(),
        totalConnections: activeConnections.size,
      });

      // Broadcast updated connection status
      broadcastConnectionStatus();
    });

    // Handle connection errors
    ws.on('error', (error: Error) => {
      logEvent('websocket_error', {
        error: error.message,
        connectionId,
        clientIp,
      }, 'error');

      activeConnections.delete(ws);
      broadcastConnectionStatus();
    });

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

/**
 * Handles set_vel WebSocket messages
 * Requirements: 2.1 - WebSocket communication for velocity commands
 */
async function handleSetVelMessage(
  ws: WebSocket, 
  message: SetVelMessage, 
  connection: WebSocketConnection
): Promise<void> {
  try {
    const velocityCommand: VelocityCommand = {
      ...message.data,
      timestamp: new Date(),
      source: 'websocket'
    };

    logEvent('websocket_set_vel_received', {
      connectionId: connection.id,
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
        velocityCommand.vx, 
        velocityCommand.vy, 
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

    // Send success response to the sender
    const successResponse: WebSocketMessage = {
      type: 'set_vel',
      data: {
        success: true,
        vx: velocityCommand.vx,
        vy: velocityCommand.vy,
        levels: velocityCommand.levels,
        timestamp: velocityCommand.timestamp?.toISOString()
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
    });

    // TODO: Forward command to fleet server (will be implemented in task 3.5)

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

/**
 * Performs heartbeat check on all connections
 * Requirements: 2.4, 2.5 - Connection monitoring and status indication
 */
function performHeartbeatCheck(): void {
  const now = new Date();
  const deadConnections: WebSocket[] = [];

  activeConnections.forEach((connection, ws) => {
    if (!connection.isAlive) {
      // Connection didn't respond to previous ping
      deadConnections.push(ws);
      return;
    }

    // Check if connection is stale (no pong response within timeout)
    const timeSinceLastPing = now.getTime() - connection.lastPing.getTime();
    if (timeSinceLastPing > HEARTBEAT_TIMEOUT) {
      deadConnections.push(ws);
      return;
    }

    // Mark as not alive and send ping
    connection.isAlive = false;
    ws.isAlive = false;
    
    try {
      ws.ping();
    } catch (error) {
      logEvent('websocket_ping_error', {
        connectionId: connection.id,
        error: error instanceof Error ? error.message : 'Unknown error',
      }, 'error');
      deadConnections.push(ws);
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