/**
 * WebSocket type extensions for heartbeat functionality
 */

import { WebSocket } from 'ws';

declare module 'ws' {
  interface WebSocket {
    isAlive?: boolean;
  }
}