/**
 * WebSocket Server Tests - Final Version
 * Tests for real-time communication functionality with proper cleanup
 */

import { WebSocketServer, WebSocket } from 'ws';
import { createServer, Server } from 'http';

// Mock dependencies
jest.mock('../../middleware/requestLogger', () => ({
  logEvent: jest.fn(),
}));

jest.mock('../../services/velocityProcessor', () => ({
  velocityProcessor: {
    validateCommand: jest.fn(),
  },
}));

jest.mock('../../database/models', () => ({
  createDatabaseService: jest.fn(),
}));

// Import after mocking
import { 
  setupWebSocket, 
  getActiveConnectionCount, 
  getConnectionStats,
  broadcastMessage
} from '../server';
import { 
  WebSocketMessage, 
  SetVelMessage, 
  VelocityCommand 
} from '@web-teleop-robot/shared';

describe('WebSocket Server', () => {
  let server: Server;
  let wss: WebSocketServer;
  let wsUrl: string;

  beforeEach(async () => {
    server = createServer();
    wss = new WebSocketServer({ server });
    setupWebSocket(wss);
    
    await new Promise<void>((resolve) => {
      server.listen(0, () => {
        const address = server.address();
        const port = typeof address === 'object' && address ? address.port : 0;
        wsUrl = `ws://localhost:${port}`;
        resolve();
      });
    });
  });

  afterEach(async () => {
    // Force close all WebSocket connections
    wss.clients.forEach((ws) => {
      ws.terminate();
    });
    
    // Close WebSocket server
    wss.close();
    
    // Close HTTP server
    server.close();
    
    // Wait for cleanup
    await new Promise(resolve => setTimeout(resolve, 50));
  });

  describe('Basic Connection Management', () => {
    it('should accept WebSocket connections and send welcome message', async () => {
      const ws = new WebSocket(wsUrl);
      
      const result = await new Promise<{ connected: boolean; message: any }>((resolve, reject) => {
        const timeout = setTimeout(() => {
          reject(new Error('Connection timeout'));
        }, 5000);
        
        ws.on('open', () => {
          // Connection established
        });
        
        ws.on('message', (data) => {
          clearTimeout(timeout);
          try {
            const message = JSON.parse(data.toString());
            resolve({ connected: true, message });
          } catch (error) {
            reject(error);
          }
        });
        
        ws.on('error', (error) => {
          clearTimeout(timeout);
          reject(error);
        });
      });
      
      expect(result.connected).toBe(true);
      expect(result.message.type).toBe('connection_status');
      expect(result.message.data.connected).toBe(true);
      expect(result.message.data.clientCount).toBeGreaterThan(0);
      
      ws.terminate();
    }, 10000);

    it('should track active connections', async () => {
      const initialCount = getActiveConnectionCount();
      
      const ws1 = new WebSocket(wsUrl);
      const ws2 = new WebSocket(wsUrl);
      
      // Wait for both connections
      await Promise.all([
        new Promise<void>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Connection 1 timeout')), 5000);
          ws1.on('open', () => { clearTimeout(timeout); resolve(); });
          ws1.on('error', reject);
        }),
        new Promise<void>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Connection 2 timeout')), 5000);
          ws2.on('open', () => { clearTimeout(timeout); resolve(); });
          ws2.on('error', reject);
        })
      ]);
      
      // Wait a bit for server to process connections
      await new Promise(resolve => setTimeout(resolve, 100));
      
      expect(getActiveConnectionCount()).toBe(initialCount + 2);
      
      ws1.terminate();
      ws2.terminate();
    }, 10000);
  });

  describe('Message Handling', () => {
    let ws: WebSocket;

    beforeEach(async () => {
      ws = new WebSocket(wsUrl);
      
      // Wait for connection and consume welcome message
      await new Promise<void>((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('Connection timeout')), 5000);
        
        ws.on('open', () => {
          // Wait for welcome message
          ws.on('message', () => {
            clearTimeout(timeout);
            resolve();
          });
        });
        
        ws.on('error', (error) => {
          clearTimeout(timeout);
          reject(error);
        });
      });
    });

    afterEach(() => {
      if (ws.readyState === WebSocket.OPEN) {
        ws.terminate();
      }
    });

    it('should handle connection_status requests', async () => {
      const statusRequest: WebSocketMessage = {
        type: 'connection_status',
        data: {},
        timestamp: new Date().toISOString(),
      };

      ws.send(JSON.stringify(statusRequest));

      const response = await new Promise<any>((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('Response timeout')), 5000);
        
        ws.on('message', (data) => {
          clearTimeout(timeout);
          try {
            const message = JSON.parse(data.toString());
            resolve(message);
          } catch (error) {
            reject(error);
          }
        });
      });

      expect(response.type).toBe('connection_status');
      expect(response.data.connected).toBe(true);
    }, 10000);

    it('should handle invalid JSON messages', async () => {
      ws.send('invalid json');

      const response = await new Promise<any>((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('Response timeout')), 5000);
        
        ws.on('message', (data) => {
          clearTimeout(timeout);
          try {
            const message = JSON.parse(data.toString());
            resolve(message);
          } catch (error) {
            reject(error);
          }
        });
      });

      expect(response.type).toBe('error');
      expect(response.data.message).toBe('Invalid message format');
    }, 10000);
  });

  describe('Velocity Command Processing', () => {
    const { velocityProcessor } = require('../../services/velocityProcessor');
    const { createDatabaseService } = require('../../database/models');

    beforeEach(() => {
      jest.clearAllMocks();
    });

    it('should process valid velocity commands', async () => {
      // Mock successful validation
      velocityProcessor.validateCommand.mockReturnValue({
        isValid: true,
        errors: [],
      });

      // Mock database service
      const mockInsertLog = jest.fn().mockResolvedValue(undefined);
      createDatabaseService.mockResolvedValue({
        getVelocityLogModel: () => ({
          insertLog: mockInsertLog,
        }),
      });

      const ws = new WebSocket(wsUrl);
      
      // Wait for connection and consume welcome message
      await new Promise<void>((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('Connection timeout')), 5000);
        
        ws.on('open', () => {
          ws.on('message', () => {
            clearTimeout(timeout);
            resolve();
          });
        });
        
        ws.on('error', reject);
      });

      const velocityCommand: VelocityCommand = {
        vx: 1.0,
        vy: 0.5,
        levels: { up: 5, down: 0, left: 0, right: 2 },
      };

      const setVelMessage: SetVelMessage = {
        type: 'set_vel',
        data: velocityCommand,
        timestamp: new Date().toISOString(),
      };

      ws.send(JSON.stringify(setVelMessage));

      const response = await new Promise<any>((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('Response timeout')), 5000);
        
        ws.on('message', (data) => {
          clearTimeout(timeout);
          try {
            const message = JSON.parse(data.toString());
            resolve(message);
          } catch (error) {
            reject(error);
          }
        });
      });

      expect(response.type).toBe('set_vel');
      expect(response.data.success).toBe(true);
      expect(response.data.vx).toBe(1.0);
      expect(response.data.vy).toBe(0.5);
      expect(velocityProcessor.validateCommand).toHaveBeenCalled();
      expect(mockInsertLog).toHaveBeenCalledWith(1.0, 0.5, velocityCommand.levels, 'websocket');
      
      ws.terminate();
    }, 10000);
  });

  describe('Broadcasting', () => {
    it('should broadcast messages to all clients', async () => {
      const ws1 = new WebSocket(wsUrl);
      const ws2 = new WebSocket(wsUrl);
      
      // Wait for both connections to be established
      await Promise.all([
        new Promise<void>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Connection 1 timeout')), 5000);
          ws1.on('open', () => {
            clearTimeout(timeout);
            resolve();
          });
          ws1.on('error', reject);
        }),
        new Promise<void>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Connection 2 timeout')), 5000);
          ws2.on('open', () => {
            clearTimeout(timeout);
            resolve();
          });
          ws2.on('error', reject);
        })
      ]);

      // Wait a bit for any initial messages to be sent
      await new Promise(resolve => setTimeout(resolve, 100));

      const testMessage: WebSocketMessage = {
        type: 'velocity_update',
        data: { test: 'broadcast' },
        timestamp: new Date().toISOString(),
      };

      // Set up message listeners that filter for our specific test message
      const messagePromises = [
        new Promise<any>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Message 1 timeout')), 5000);
          
          const messageHandler = (data: Buffer) => {
            try {
              const message = JSON.parse(data.toString());
              // Only resolve when we get our specific test message
              if (message.type === 'velocity_update' && message.data && message.data.test === 'broadcast') {
                clearTimeout(timeout);
                ws1.off('message', messageHandler);
                resolve(message);
              }
            } catch (error) {
              // Ignore parsing errors, continue listening
            }
          };
          
          ws1.on('message', messageHandler);
        }),
        new Promise<any>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Message 2 timeout')), 5000);
          
          const messageHandler = (data: Buffer) => {
            try {
              const message = JSON.parse(data.toString());
              // Only resolve when we get our specific test message
              if (message.type === 'velocity_update' && message.data && message.data.test === 'broadcast') {
                clearTimeout(timeout);
                ws2.off('message', messageHandler);
                resolve(message);
              }
            } catch (error) {
              // Ignore parsing errors, continue listening
            }
          };
          
          ws2.on('message', messageHandler);
        })
      ];

      // Broadcast the message
      broadcastMessage(testMessage);

      const responses = await Promise.all(messagePromises);

      responses.forEach((response) => {
        expect(response.type).toBe('velocity_update');
        expect(response.data.test).toBe('broadcast');
      });
      
      ws1.terminate();
      ws2.terminate();
    }, 15000);
  });

  describe('Connection Statistics', () => {
    it('should return accurate connection statistics', async () => {
      const ws1 = new WebSocket(wsUrl);
      const ws2 = new WebSocket(wsUrl);
      
      // Wait for connections
      await Promise.all([
        new Promise<void>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Connection 1 timeout')), 5000);
          ws1.on('open', () => { clearTimeout(timeout); resolve(); });
          ws1.on('error', reject);
        }),
        new Promise<void>((resolve, reject) => {
          const timeout = setTimeout(() => reject(new Error('Connection 2 timeout')), 5000);
          ws2.on('open', () => { clearTimeout(timeout); resolve(); });
          ws2.on('error', reject);
        })
      ]);

      // Wait for server to process connections
      await new Promise(resolve => setTimeout(resolve, 100));

      const stats = getConnectionStats();
      
      expect(stats.totalConnections).toBe(2);
      expect(stats.connections).toHaveLength(2);
      
      stats.connections.forEach((conn) => {
        expect(conn.id).toBeDefined();
        expect(conn.clientIp).toBeDefined();
        expect(conn.connectedAt).toBeDefined();
        expect(conn.lastPing).toBeDefined();
        expect(typeof conn.isAlive).toBe('boolean');
        expect(typeof conn.connectedDuration).toBe('number');
      });
      
      ws1.terminate();
      ws2.terminate();
    }, 10000);
  });
});