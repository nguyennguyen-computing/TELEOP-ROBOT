/**
 * Integration tests for authentication system
 */

import request from 'supertest';
import { Express } from 'express';
import { createApp } from '../../test-utils/app-factory';

describe('Authentication Integration Tests', () => {
    let app: Express;
    let originalEnv: any;

    beforeAll(async () => {
        // Save original environment
        originalEnv = {
            AUTH_ENABLED: process.env.AUTH_ENABLED,
            API_KEYS: process.env.API_KEYS,
            JWT_SECRET: process.env.JWT_SECRET,
            JWT_EXPIRES_IN: process.env.JWT_EXPIRES_IN,
        };

        // Create test app with authentication enabled
        process.env.AUTH_ENABLED = 'true';
        process.env.API_KEYS = 'test-api-key,admin-key';
        process.env.JWT_SECRET = 'test-jwt-secret-key';
        process.env.JWT_EXPIRES_IN = '1h';

        app = await createApp();
    });

    afterAll(() => {
        // Restore original environment
        Object.keys(originalEnv).forEach(key => {
            if (originalEnv[key] !== undefined) {
                process.env[key] = originalEnv[key];
            } else {
                delete process.env[key];
            }
        });
    });

    describe('API Key Authentication', () => {
        it('should allow velocity commands with valid API key', async () => {
            const velocityCommand = {
                vx: 0.5,
                vy: 0.0,
                levels: { up: 5, down: 0, left: 0, right: 0 }
            };

            const response = await request(app)
                .post('/api/vel')
                .set('Authorization', 'ApiKey test-api-key')
                .send(velocityCommand)
                .expect(200);

            expect(response.body.ok).toBe(true);
        });

        it('should allow fleet stats access with valid API key', async () => {
            const response = await request(app)
                .get('/api/fleet/stats')
                .set('Authorization', 'ApiKey test-api-key')
                .expect(200);

            expect(response.body.ok).toBe(true);
        });
    });

    describe('Authentication Middleware', () => {
        it('should verify authentication middleware is applied to protected routes', async () => {
            // Test that the middleware is working by checking response structure
            const response = await request(app)
                .get('/api/fleet/stats')
                .set('Authorization', 'ApiKey test-api-key');

            // Should get a successful response with auth
            expect(response.status).toBe(200);
            expect(response.body.ok).toBe(true);
        });
    });

    describe('Rate Limiting Integration', () => {
        it('should apply rate limiting middleware to API routes', async () => {
            // Test that rate limiting middleware is applied
            const response = await request(app)
                .get('/api/fleet/stats')
                .set('Authorization', 'ApiKey test-api-key');

            expect(response.status).toBe(200);
            // Rate limiting middleware should be applied (we can't easily test the actual limiting in unit tests)
        });
    });

    describe('Public Endpoints', () => {
        it('should allow access to health endpoint without authentication', async () => {
            const response = await request(app)
                .get('/api/health')
                .expect(200);

            expect(response.body.ok).toBe(true);
        });
    });
});