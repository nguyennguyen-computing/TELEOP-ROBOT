import { MongoClient, Db, MongoClientOptions } from 'mongodb';

export class DatabaseConnection {
  private static instance: DatabaseConnection;
  private client: MongoClient | null = null;
  private db: Db | null = null;
  private connectionString: string;
  private dbName: string;
  private isConnecting = false;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 5;
  private reconnectDelay = 1000; // Start with 1 second

  private constructor(connectionString: string, dbName: string) {
    this.connectionString = connectionString;
    this.dbName = dbName;
  }

  /**
   * Get singleton instance of database connection
   */
  public static getInstance(connectionString?: string, dbName?: string): DatabaseConnection {
    if (!DatabaseConnection.instance) {
      if (!connectionString || !dbName) {
        throw new Error('Connection string and database name required for first initialization');
      }
      DatabaseConnection.instance = new DatabaseConnection(connectionString, dbName);
    }
    return DatabaseConnection.instance;
  }

  /**
   * Connect to MongoDB with connection pooling and retry logic
   */
  public async connect(): Promise<void> {
    if (this.client && this.db) {
      return; // Already connected
    }

    if (this.isConnecting) {
      // Wait for existing connection attempt
      while (this.isConnecting) {
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      return;
    }

    this.isConnecting = true;

    try {
      const options: MongoClientOptions = {
        // Connection pooling configuration
        maxPoolSize: 10,
        minPoolSize: 2,
        maxIdleTimeMS: 30000,
        
        // Connection timeout settings
        serverSelectionTimeoutMS: 5000,
        connectTimeoutMS: 10000,
        socketTimeoutMS: 45000,
        
        // Retry configuration
        retryWrites: true,
        retryReads: true,
        
        // Monitoring
        monitorCommands: process.env.NODE_ENV === 'development',
      };

      console.log(`Connecting to MongoDB: ${this.connectionString.replace(/\/\/.*@/, '//***:***@')}`);
      
      this.client = new MongoClient(this.connectionString, options);
      await this.client.connect();
      
      this.db = this.client.db(this.dbName);
      
      // Test the connection
      await this.db.admin().ping();
      
      console.log(`Successfully connected to MongoDB database: ${this.dbName}`);
      
      // Reset reconnection attempts on successful connection
      this.reconnectAttempts = 0;
      this.reconnectDelay = 1000;
      
      // Set up connection event listeners
      this.setupEventListeners();
      
    } catch (error) {
      console.error('Failed to connect to MongoDB:', error);
      await this.handleConnectionError(error as Error);
    } finally {
      this.isConnecting = false;
    }
  }

  /**
   * Get database instance
   */
  public getDb(): Db {
    if (!this.db) {
      throw new Error('Database not connected. Call connect() first.');
    }
    return this.db;
  }

  /**
   * Get MongoDB client instance
   */
  public getClient(): MongoClient {
    if (!this.client) {
      throw new Error('Database client not initialized. Call connect() first.');
    }
    return this.client;
  }

  /**
   * Check if database is connected
   */
  public isConnected(): boolean {
    return this.client !== null && this.db !== null;
  }

  /**
   * Disconnect from MongoDB
   */
  public async disconnect(): Promise<void> {
    if (this.client) {
      console.log('Disconnecting from MongoDB...');
      await this.client.close();
      this.client = null;
      this.db = null;
      console.log('Disconnected from MongoDB');
    }
  }

  /**
   * Handle connection errors with exponential backoff retry
   */
  private async handleConnectionError(error: Error): Promise<void> {
    this.reconnectAttempts++;
    
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error(`Failed to connect to MongoDB after ${this.maxReconnectAttempts} attempts`);
      throw error;
    }

    console.log(`Connection attempt ${this.reconnectAttempts} failed. Retrying in ${this.reconnectDelay}ms...`);
    
    await new Promise(resolve => setTimeout(resolve, this.reconnectDelay));
    
    // Exponential backoff with jitter
    this.reconnectDelay = Math.min(this.reconnectDelay * 2 + Math.random() * 1000, 30000);
    
    // Retry connection
    await this.connect();
  }

  /**
   * Set up MongoDB client event listeners
   */
  private setupEventListeners(): void {
    if (!this.client) return;

    this.client.on('connectionPoolCreated', () => {
      console.log('MongoDB connection pool created');
    });

    this.client.on('connectionPoolClosed', () => {
      console.log('MongoDB connection pool closed');
    });

    this.client.on('serverHeartbeatFailed', (event) => {
      console.warn('MongoDB server heartbeat failed:', event);
    });

    this.client.on('topologyDescriptionChanged', (event) => {
      if (process.env.NODE_ENV === 'development') {
        console.log('MongoDB topology changed:', event.newDescription.type);
      }
    });
  }

  /**
   * Health check for database connection
   */
  public async healthCheck(): Promise<{ connected: boolean; latency?: number; error?: string }> {
    try {
      if (!this.db) {
        return { connected: false, error: 'Database not initialized' };
      }

      const start = Date.now();
      await this.db.admin().ping();
      const latency = Date.now() - start;

      return { connected: true, latency };
    } catch (error) {
      return { 
        connected: false, 
        error: error instanceof Error ? error.message : 'Unknown error' 
      };
    }
  }

  /**
   * Get connection statistics
   */
  public getConnectionStats() {
    if (!this.client) {
      return null;
    }

    return {
      isConnected: this.isConnected(),
      reconnectAttempts: this.reconnectAttempts,
      dbName: this.dbName,
    };
  }
}

/**
 * Initialize database connection with environment configuration
 */
export async function initializeDatabase(): Promise<DatabaseConnection> {
  const mongoUri = process.env.MONGO_URI || 'mongodb://localhost:27017/teleop_db';
  const dbName = process.env.MONGO_DB || 'teleop_db';
  
  const dbConnection = DatabaseConnection.getInstance(mongoUri, dbName);
  await dbConnection.connect();
  
  return dbConnection;
}

/**
 * Connect to database and return database instance
 * Convenience function for backward compatibility
 */
export async function connectToDatabase(): Promise<Db> {
  const dbConnection = await initializeDatabase();
  return dbConnection.getDb();
}

/**
 * Graceful shutdown handler for database connection
 */
export async function gracefulShutdown(): Promise<void> {
  try {
    const dbConnection = DatabaseConnection.getInstance();
    await dbConnection.disconnect();
  } catch (error) {
    console.error('Error during database shutdown:', error);
  }
}