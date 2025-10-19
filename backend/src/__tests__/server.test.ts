/**
 * Tests for Express server setup and middleware
 */

import request from 'supertest';

// Mock the database connection before importing the app
jest.mock('../database/connection', () => ({
  connectToDatabase: jest.fn().mockResolvedValue({
    admin: () => ({
      ping: jest.fn().mockResolvedValue(true),
    }),
  }),
}));

import { app } from '../index';

describe('Express Server Setup', () => {
  describe('Health Check Endpoints', () => {
    it('should respond to /health endpoint', async () => {
      const response = await request(app)
        .get('/health')
        .expect(200);

      expect(response.body).toMatchObject({
        ok: true,
        service: 'web-teleop-robot-backend',
        timestamp: expect.any(String),
        uptime: expect.any(Number),
        version: expect.any(String),
      });
    });

    it('should respond to /health/ready endpoint', async () => {
      const response = await request(app)
        .get('/health/ready')
        .expect('Content-Type', /json/);

      // Should be either 200 (healthy) or 503 (unhealthy)
      expect([200, 503]).toContain(response.status);
      expect(response.body).toHaveProperty('ok');
      expect(response.body).toHaveProperty('service', 'web-teleop-robot-backend');
      expect(response.body).toHaveProperty('timestamp');
      expect(response.body).toHaveProperty('checks');
    });
  });

  describe('API Routes', () => {
    it('should respond to /api/health endpoint', async () => {
      const response = await request(app)
        .get('/api/health')
        .expect(200);

      expect(response.body).toMatchObject({
        ok: true,
        data: {
          service: 'api',
          timestamp: expect.any(String),
          version: '1.0.0',
        },
        timestamp: expect.any(String),
      });
    });

    it('should return 400 for invalid /api/vel request', async () => {
      const response = await request(app)
        .post('/api/vel')
        .send({ vx: 0.5, vy: 0.0 }) // Missing levels
        .expect(400);

      expect(response.body).toMatchObject({
        ok: false,
        error: expect.stringContaining('Invalid request body'),
        timestamp: expect.any(String),
      });
    });

    it('should return logs from /api/logs endpoint', async () => {
      const response = await request(app)
        .get('/api/logs');

      // Should return 200 or 500 depending on database connection
      expect([200, 500]).toContain(response.status);
      expect(response.body).toMatchObject({
        ok: expect.any(Boolean),
        timestamp: expect.any(String),
      });
    });
  });

  describe('Middleware', () => {
    it('should set security headers', async () => {
      const response = await request(app)
        .get('/health')
        .expect(200);

      // Check for Helmet security headers
      expect(response.headers).toHaveProperty('x-content-type-options');
      expect(response.headers).toHaveProperty('x-frame-options');
      expect(response.headers).toHaveProperty('x-xss-protection');
    });

    it('should set CORS headers', async () => {
      const response = await request(app)
        .options('/api/health')
        .set('Origin', process.env.CORS_ORIGIN || 'http://localhost:3000')
        .expect(204);

      expect(response.headers).toHaveProperty('access-control-allow-origin');
    });

    it('should add correlation ID to responses', async () => {
      const response = await request(app)
        .get('/health')
        .expect(200);

      expect(response.headers).toHaveProperty('x-correlation-id');
      expect(response.headers['x-correlation-id']).toMatch(/^[0-9a-f-]{36}$/);
    });

    it('should handle JSON parsing errors', async () => {
      const response = await request(app)
        .post('/api/vel')
        .set('Content-Type', 'application/json')
        .send('invalid json')
        .expect(400);

      expect(response.body).toMatchObject({
        ok: false,
        error: expect.stringContaining('JSON'),
        timestamp: expect.any(String),
      });
    });
  });

  describe('Error Handling', () => {
    it('should return 404 for unknown routes', async () => {
      const response = await request(app)
        .get('/unknown-route')
        .expect(404);

      expect(response.body).toMatchObject({
        ok: false,
        error: expect.stringContaining('not found'),
        timestamp: expect.any(String),
      });
    });

    it('should handle large request bodies', async () => {
      const largePayload = 'x'.repeat(11 * 1024 * 1024); // 11MB

      const response = await request(app)
        .post('/api/vel')
        .send({ data: largePayload });

      // Should be either 413 (Payload Too Large) or 400 (Bad Request)
      expect([400, 413, 500]).toContain(response.status);
      expect(response.body).toHaveProperty('ok', false);
    });
  });

  describe('Request Logging', () => {
    it('should log requests with structured format', async () => {
      // Capture console output
      const consoleSpy = jest.spyOn(console, 'log').mockImplementation();

      await request(app)
        .get('/health')
        .expect(200);

      // Check that structured logs were created
      expect(consoleSpy).toHaveBeenCalled();
      
      // Find log entries that look like structured logs
      const logCalls = consoleSpy.mock.calls.filter(call => {
        try {
          const logEntry = JSON.parse(call[0]);
          return logEntry.service === 'node-api' && logEntry.correlationId;
        } catch {
          return false;
        }
      });

      expect(logCalls.length).toBeGreaterThan(0);

      consoleSpy.mockRestore();
    });
  });
});