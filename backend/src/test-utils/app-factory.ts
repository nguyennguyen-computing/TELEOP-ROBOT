/**
 * Test utility for creating Express app instances for testing
 */

import express from 'express';
import cors from 'cors';
import { createApiRoutes } from '../routes/api';
import { errorHandler, notFoundHandler } from '../middleware/errorHandler';

/**
 * Creates a minimal Express app for testing API routes
 */
export async function createApp() {
  const app = express();

  // Basic middleware
  app.use(cors());
  app.use(express.json());
  app.use(express.urlencoded({ extended: true }));

  // API routes
  app.use('/api', createApiRoutes());

  // Error handling
  app.use(notFoundHandler);
  app.use(errorHandler);

  return app;
}