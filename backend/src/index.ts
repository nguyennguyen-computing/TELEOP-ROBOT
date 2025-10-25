/**
 * Main Express.js server for Web Teleop Robot backend API
 * Handles velocity commands, logging, and WebSocket communication
 */

import express from 'express';
import cors from 'cors';
import helmet from 'helmet';
import morgan from 'morgan';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import dotenv from 'dotenv';
import { connectToDatabase } from './database/connection';
import { createApiRoutes } from './routes/api';
import { setupWebSocket } from './websocket/server';
import { errorHandler, notFoundHandler } from './middleware/errorHandler';
import { requestLogger } from './middleware/requestLogger';
import { validateEnvironment } from './config/environment';


// Load environment variables
const dotenvResult = dotenv.config({ path: '.env' });
console.log('Dotenv result:', dotenvResult);
console.log('NODE_PORT from env:', process.env.NODE_PORT);
console.log('AUTH_ENABLED from env:', process.env.AUTH_ENABLED);

// Validate environment configuration
const config = validateEnvironment();

// Create Express application
const app = express();
const server = createServer(app);

// Security middleware
app.use(helmet({
  contentSecurityPolicy: {
    directives: {
      defaultSrc: ["'self'"],
      styleSrc: ["'self'", "'unsafe-inline'"],
      scriptSrc: ["'self'"],
      imgSrc: ["'self'", "data:", "https:"],
    },
  },
  crossOriginEmbedderPolicy: false,
}));

// CORS configuration
app.use(cors({
  origin: config.CORS_ORIGIN,
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization', 'X-Requested-With'],
}));

// Body parsing middleware
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));

// Request logging middleware
app.use(morgan('combined', {
  stream: {
    write: (message: string) => {
      console.log(message.trim());
    }
  }
}));

// Custom request logger for structured logging
app.use(requestLogger);

// Health check endpoints
app.get('/health', (_req, res) => {
  res.status(200).json({
    ok: true,
    service: 'web-teleop-robot-backend',
    timestamp: new Date().toISOString(),
    uptime: process.uptime(),
    version: process.env.npm_package_version || '1.0.0',
  });
});

app.get('/health/ready', async (_req, res) => {
  try {
    // Check database connection
    const db = await connectToDatabase();
    await db.admin().ping();
    
    res.status(200).json({
      ok: true,
      service: 'web-teleop-robot-backend',
      timestamp: new Date().toISOString(),
      checks: {
        database: 'healthy',
      },
    });
  } catch (error) {
    res.status(503).json({
      ok: false,
      service: 'web-teleop-robot-backend',
      timestamp: new Date().toISOString(),
      checks: {
        database: 'unhealthy',
      },
      error: error instanceof Error ? error.message : 'Unknown error',
    });
  }
});

// API routes
app.use('/api', createApiRoutes());

// 404 handler
app.use(notFoundHandler);

// Error handling middleware (must be last)
app.use(errorHandler);

// Setup WebSocket server
const wss = new WebSocketServer({ server });
setupWebSocket(wss);

// Start server
const startServer = async () => {
  try {
    // Connect to database
    console.log('Connecting to database...');
    await connectToDatabase();
    console.log('Database connected successfully');

    // Start HTTP server
    server.listen(config.NODE_PORT, () => {
      console.log(`🚀 Server running on port ${config.NODE_PORT}`);
      console.log(`📡 WebSocket server ready`);
      console.log(`🏥 Health check: http://localhost:${config.NODE_PORT}/health`);
      console.log(`🔧 Environment: ${process.env.NODE_ENV || 'development'}`);
    });

    // Graceful shutdown handling
    const gracefulShutdown = (signal: string) => {
      console.log(`\n${signal} received. Starting graceful shutdown...`);
      
      server.close(() => {
        console.log('HTTP server closed');
        process.exit(0);
      });

      // Force close after 10 seconds
      setTimeout(() => {
        console.error('Could not close connections in time, forcefully shutting down');
        process.exit(1);
      }, 10000);
    };

    process.on('SIGTERM', () => gracefulShutdown('SIGTERM'));
    process.on('SIGINT', () => gracefulShutdown('SIGINT'));

  } catch (error) {
    console.error('Failed to start server:', error);
    process.exit(1);
  }
};

// Start the server
startServer();

export { app, server };