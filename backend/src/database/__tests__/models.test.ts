/**
 * Unit tests for database models and operations
 */

import { MongoMemoryServer } from 'mongodb-memory-server';
import { MongoClient, Db } from 'mongodb';
import { VelocityLogModel } from '../models';
import { SpeedLevels } from '@web-teleop-robot/shared';

describe('VelocityLogModel', () => {
    let mongoServer: MongoMemoryServer;
    let client: MongoClient;
    let db: Db;
    let velocityLogModel: VelocityLogModel;

    beforeAll(async () => {
        mongoServer = await MongoMemoryServer.create();
        const uri = mongoServer.getUri();
        client = new MongoClient(uri);
        await client.connect();
        db = client.db('test_teleop_db');
        velocityLogModel = new VelocityLogModel(db);
        await velocityLogModel.ensureIndexes();
    });

    afterAll(async () => {
        await client.close();
        await mongoServer.stop();
    });

    beforeEach(async () => {
        // Clean up collection before each test
        await db.collection('vel_logs').deleteMany({});
    });

    describe('insertLog', () => {
        it('should insert a valid velocity log entry', async () => {
            const levels: SpeedLevels = { up: 5, down: 0, left: 2, right: 0 };
            const result = await velocityLogModel.insertLog(0.5, 0.2, levels, 'web');

            expect(result.insertedId).toBeDefined();
            expect(result.acknowledged).toBe(true);
        });

        it('should reject invalid velocity values', async () => {
            const levels: SpeedLevels = { up: 5, down: 0, left: 2, right: 0 };

            await expect(
                velocityLogModel.insertLog(NaN, 0.2, levels, 'web')
            ).rejects.toThrow('Invalid log entry');
        });

        it('should reject invalid level values', async () => {
            const invalidLevels: SpeedLevels = { up: 15, down: 0, left: 2, right: 0 }; // up > 10

            await expect(
                velocityLogModel.insertLog(0.5, 0.2, invalidLevels, 'web')
            ).rejects.toThrow('Invalid log entry');
        });
    });

    describe('getLogs', () => {
        beforeEach(async () => {
            // Insert test data
            const testLevels: SpeedLevels = { up: 3, down: 0, left: 1, right: 0 };
            await velocityLogModel.insertLog(0.3, 0.1, testLevels, 'web');
            await velocityLogModel.insertLog(0.5, 0.0, testLevels, 'web');
            await velocityLogModel.insertLog(-0.2, 0.3, testLevels, 'web');
        });

        it('should retrieve logs with default pagination', async () => {
            const logs = await velocityLogModel.getLogs();

            expect(logs).toHaveLength(3);
            expect(logs[0].ts.getTime()).toBeGreaterThanOrEqual(logs[1].ts.getTime()); // Newest first
        });

        it('should respect limit parameter', async () => {
            const logs = await velocityLogModel.getLogs({ limit: 2 });

            expect(logs).toHaveLength(2);
        });

        it('should filter by source', async () => {
            await velocityLogModel.insertLog(0.1, 0.1, { up: 1, down: 0, left: 0, right: 0 }, 'test');

            const webLogs = await velocityLogModel.getLogs({ source: 'web' });
            const testLogs = await velocityLogModel.getLogs({ source: 'test' });

            expect(webLogs).toHaveLength(3);
            expect(testLogs).toHaveLength(1);
        });
    });

    describe('getLatestLog', () => {
        it('should return null when no logs exist', async () => {
            const latest = await velocityLogModel.getLatestLog();
            expect(latest).toBeNull();
        });

        it('should return the most recent log', async () => {
            const levels: SpeedLevels = { up: 2, down: 0, left: 0, right: 1 };

            await velocityLogModel.insertLog(0.1, 0.1, levels, 'web');
            await new Promise(resolve => setTimeout(resolve, 10)); // Small delay
            await velocityLogModel.insertLog(0.2, 0.2, levels, 'web');

            const latest = await velocityLogModel.getLatestLog();

            expect(latest).toBeDefined();
            expect(latest!.vx).toBe(0.2);
            expect(latest!.vy).toBe(0.2);
        });
    });

    describe('getLogsCount', () => {
        beforeEach(async () => {
            const levels: SpeedLevels = { up: 1, down: 0, left: 0, right: 0 };
            await velocityLogModel.insertLog(0.1, 0.0, levels, 'web');
            await velocityLogModel.insertLog(0.2, 0.0, levels, 'web');
            await velocityLogModel.insertLog(0.3, 0.0, levels, 'test');
        });

        it('should count all logs when no filter provided', async () => {
            const count = await velocityLogModel.getLogsCount();
            expect(count).toBe(3);
        });

        it('should count logs by source filter', async () => {
            const webCount = await velocityLogModel.getLogsCount({ source: 'web' });
            const testCount = await velocityLogModel.getLogsCount({ source: 'test' });

            expect(webCount).toBe(2);
            expect(testCount).toBe(1);
        });
    });

    describe('deleteOldLogs', () => {
        it('should delete logs older than specified date', async () => {
            const levels: SpeedLevels = { up: 1, down: 0, left: 0, right: 0 };

            // Insert old log
            await velocityLogModel.insertLog(0.1, 0.0, levels, 'web');

            // Wait a small amount to ensure timestamp difference
            await new Promise(resolve => setTimeout(resolve, 10));

            // Delete logs older than now
            const deletedCount = await velocityLogModel.deleteOldLogs(new Date());

            expect(deletedCount).toBe(1);

            const remainingCount = await velocityLogModel.getLogsCount();
            expect(remainingCount).toBe(0);
        });
    });
});