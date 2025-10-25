import { Collection, Db, InsertOneResult, FindOptions, CreateIndexesOptions } from 'mongodb';
import { VelocityLogEntry, SpeedLevels, ValidationResult } from '@web-teleop-robot/shared';
import { DatabaseConnection } from './connection';

/**
 * Velocity Log Model for database operations
 */
export class VelocityLogModel {
  private collection: Collection<VelocityLogEntry>;
  private db: Db;

  constructor(db: Db) {
    this.db = db;
    this.collection = db.collection<VelocityLogEntry>('vel_logs');
  }

  async insertLog(
    vx: number,
    vy: number,
    levels: SpeedLevels,
    source: string = 'web'
  ): Promise<InsertOneResult<VelocityLogEntry>> {
    const logEntry: VelocityLogEntry = {
      ts: new Date(),
      vx,
      vy,
      levels: { ...levels }, // Create a copy to avoid reference issues
      source
    };

    // Validate the log entry before insertion
    const validation = this.validateLogEntry(logEntry);
    if (!validation.isValid) {
      throw new Error(`Invalid log entry: ${validation.errors.join(', ')}`);
    }

    try {
      const result = await this.collection.insertOne(logEntry);
      console.log(`Velocity log inserted with ID: ${result.insertedId}`);
      return result;
    } catch (error) {
      console.error('Failed to insert velocity log:', error);
      throw new Error(`Database insertion failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  async getLogs(options: {
    limit?: number;
    skip?: number;
    source?: string;
    startDate?: Date;
    endDate?: Date;
    sortBy?: 'ts' | 'vx' | 'vy';
    sortOrder?: 1 | -1;
  } = {}): Promise<VelocityLogEntry[]> {
    const {
      limit = 100,
      skip = 0,
      source,
      startDate,
      endDate,
      sortBy = 'ts',
      sortOrder = -1 // Default to descending (newest first)
    } = options;

    // Build query filter
    const filter: any = {};
    
    if (source) {
      filter.source = source;
    }

    if (startDate || endDate) {
      filter.ts = {};
      if (startDate) filter.ts.$gte = startDate;
      if (endDate) filter.ts.$lte = endDate;
    }

    // Build find options
    const findOptions: FindOptions = {
      limit: Math.min(limit, 1000), // Cap at 1000 for performance
      skip,
      sort: { [sortBy]: sortOrder }
    };

    try {
      const logs = await this.collection.find(filter, findOptions).toArray();
      console.log(`Retrieved ${logs.length} velocity logs`);
      return logs;
    } catch (error) {
      console.error('Failed to query velocity logs:', error);
      throw new Error(`Database query failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Get the latest velocity log entry
   */
  async getLatestLog(): Promise<VelocityLogEntry | null> {
    try {
      const result = await this.collection.findOne({}, { sort: { ts: -1 } });
      return result;
    } catch (error) {
      console.error('Failed to get latest velocity log:', error);
      throw new Error(`Database query failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Get logs count for pagination
   */
  async getLogsCount(filter: {
    source?: string;
    startDate?: Date;
    endDate?: Date;
  } = {}): Promise<number> {
    const { source, startDate, endDate } = filter;

    const queryFilter: any = {};
    
    if (source) {
      queryFilter.source = source;
    }

    if (startDate || endDate) {
      queryFilter.ts = {};
      if (startDate) queryFilter.ts.$gte = startDate;
      if (endDate) queryFilter.ts.$lte = endDate;
    }

    try {
      return await this.collection.countDocuments(queryFilter);
    } catch (error) {
      console.error('Failed to count velocity logs:', error);
      throw new Error(`Database count failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Delete old logs (for maintenance)
   */
  async deleteOldLogs(olderThan: Date): Promise<number> {
    try {
      const result = await this.collection.deleteMany({ ts: { $lt: olderThan } });
      console.log(`Deleted ${result.deletedCount} old velocity logs`);
      return result.deletedCount;
    } catch (error) {
      console.error('Failed to delete old velocity logs:', error);
      throw new Error(`Database deletion failed: ${error instanceof Error ? error.message : 'Unknown error'}`);
    }
  }

  /**
   * Validate velocity log entry
   */
  private validateLogEntry(entry: VelocityLogEntry): ValidationResult {
    const errors: string[] = [];

    // Validate timestamp
    if (!entry.ts || !(entry.ts instanceof Date) || isNaN(entry.ts.getTime())) {
      errors.push('Invalid timestamp');
    }

    // Validate velocities
    if (typeof entry.vx !== 'number' || isNaN(entry.vx)) {
      errors.push('Invalid vx value');
    }

    if (typeof entry.vy !== 'number' || isNaN(entry.vy)) {
      errors.push('Invalid vy value');
    }

    // Validate levels
    if (!entry.levels || typeof entry.levels !== 'object') {
      errors.push('Invalid levels object');
    } else {
      const requiredLevels = ['up', 'down', 'left', 'right'];
      for (const level of requiredLevels) {
        const value = entry.levels[level as keyof SpeedLevels];
        if (typeof value !== 'number' || isNaN(value) || value < 0 || value > 10) {
          errors.push(`Invalid ${level} level: must be a number between 0 and 10`);
        }
      }
    }

    // Validate source
    if (!entry.source || typeof entry.source !== 'string') {
      errors.push('Invalid source');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  async ensureIndexes(): Promise<void> {
    try {
      const indexes = [
        // Primary timestamp index for time-based queries (descending for latest first)
        { key: { ts: -1 }, name: 'timestamp_desc_idx' },
        
        // Compound index for source and timestamp queries
        { key: { source: 1, ts: -1 }, name: 'source_timestamp_idx' },
        
        // Index for velocity range queries (if needed for analytics)
        { key: { vx: 1, vy: 1 }, name: 'velocity_idx' }
      ];

      for (const index of indexes) {
        await this.collection.createIndex(index.key, { 
          name: index.name,
          background: true // Create index in background to avoid blocking
        } as CreateIndexesOptions);
      }

      console.log('Database indexes ensured for vel_logs collection');
    } catch (error) {
      console.error('Failed to create indexes:', error);
      // Don't throw here as the application can still function without optimal indexes
    }
  }

  /**
   * Get collection statistics
   */
  async getStats() {
    try {
      const stats = await this.db.command({ collStats: 'vel_logs' });
      return {
        count: stats.count,
        size: stats.size,
        avgObjSize: stats.avgObjSize,
        indexCount: stats.nindexes,
        totalIndexSize: stats.totalIndexSize
      };
    } catch (error) {
      console.error('Failed to get collection stats:', error);
      return null;
    }
  }
}

/**
 * Database service class that provides high-level database operations
 */
export class DatabaseService {
  private velocityLogModel: VelocityLogModel;
  private dbConnection: DatabaseConnection;

  constructor(dbConnection: DatabaseConnection) {
    this.dbConnection = dbConnection;
    this.velocityLogModel = new VelocityLogModel(dbConnection.getDb());
  }

  /**
   * Initialize database service and ensure indexes
   */
  async initialize(): Promise<void> {
    await this.velocityLogModel.ensureIndexes();
    console.log('Database service initialized');
  }

  /**
   * Get velocity log model
   */
  getVelocityLogModel(): VelocityLogModel {
    return this.velocityLogModel;
  }

  /**
   * Health check for database service
   */
  async healthCheck() {
    const connectionHealth = await this.dbConnection.healthCheck();
    const stats = await this.velocityLogModel.getStats();
    
    return {
      connection: connectionHealth,
      stats,
      timestamp: new Date().toISOString()
    };
  }

  /**
   * Graceful shutdown
   */
  async shutdown(): Promise<void> {
    await this.dbConnection.disconnect();
  }
}

/**
 * Factory function to create database service
 */
export async function createDatabaseService(): Promise<DatabaseService> {
  const dbConnection = DatabaseConnection.getInstance();
  
  if (!dbConnection.isConnected()) {
    await dbConnection.connect();
  }
  
  const service = new DatabaseService(dbConnection);
  await service.initialize();
  
  return service;
}