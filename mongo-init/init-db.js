// MongoDB initialization script for Web Teleop Robot system
// Requirements: 3.2, 3.3, 3.5

// Switch to the teleop database
db = db.getSiblingDB('teleop_db');

// Create vel_logs collection with proper schema validation
// Requirement 3.2: Use database "teleop_db" and collection "vel_logs"
// Requirement 3.3: Include fields: ts (ISODate), vx (Number), vy (Number), levels (Object), source ('web')
db.createCollection('vel_logs', {
  validator: {
    $jsonSchema: {
      bsonType: 'object',
      required: ['ts', 'vx', 'vy', 'levels', 'source'],
      properties: {
        ts: {
          bsonType: 'date',
          description: 'Timestamp must be a date and is required'
        },
        vx: {
          bsonType: 'number',
          description: 'Velocity X must be a number and is required'
        },
        vy: {
          bsonType: 'number',
          description: 'Velocity Y must be a number and is required'
        },
        levels: {
          bsonType: 'object',
          required: ['up', 'down', 'left', 'right'],
          properties: {
            up: { 
              bsonType: 'number', 
              minimum: 0, 
              maximum: 10,
              description: 'Up level must be between 0 and 10'
            },
            down: { 
              bsonType: 'number', 
              minimum: 0, 
              maximum: 10,
              description: 'Down level must be between 0 and 10'
            },
            left: { 
              bsonType: 'number', 
              minimum: 0, 
              maximum: 10,
              description: 'Left level must be between 0 and 10'
            },
            right: { 
              bsonType: 'number', 
              minimum: 0, 
              maximum: 10,
              description: 'Right level must be between 0 and 10'
            }
          },
          additionalProperties: false,
          description: 'Levels object with up/down/left/right properties is required'
        },
        source: {
          bsonType: 'string',
          enum: ['web'],
          description: 'Source must be "web"'
        }
      },
      additionalProperties: true
    }
  }
});

// Requirement 3.5: Create timestamp index { ts: -1 } for efficient queries
db.vel_logs.createIndex({ 'ts': -1 }, { name: 'timestamp_desc_idx' });

// Additional indexes for better query performance
db.vel_logs.createIndex({ 'source': 1, 'ts': -1 }, { name: 'source_timestamp_idx' });

// Create a collection for system configuration
db.createCollection('system_config');

// Insert default system configuration
db.system_config.insertOne({
  _id: 'default',
  VX_MAX: 1.0,
  VY_MAX: 1.0,
  STEP_X: 0.1,
  STEP_Y: 0.1,
  created_at: new Date(),
  updated_at: new Date()
});

print('Database initialization completed successfully');