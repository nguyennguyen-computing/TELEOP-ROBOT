/**
 * Type definitions for robot control interface
 */

export interface VelocityCommand {
  vx: number
  vy: number
  levels: {
    up: number
    down: number
    left: number
    right: number
  }
  timestamp?: Date
  source?: string
}

export interface RobotState {
  levels: {
    up: number
    down: number
    left: number
    right: number
  }
  velocity: {
    vx: number
    vy: number
  }
  connectionStatus: 'connected' | 'disconnected' | 'connecting'
  isHolding: {
    [key: string]: boolean
  }
  
  // UI/UX specific state
  theme: 'light' | 'dark'
  notifications: Notification[]
  isFullscreen: boolean
  hapticFeedback: boolean
  soundEnabled: boolean
  lastCommandTime: Date | null
  commandHistory: VelocityCommand[]
}

export interface Notification {
  id: string
  type: 'success' | 'warning' | 'error' | 'info'
  message: string
  duration?: number
  timestamp: Date
}

export type ConnectionStatus = 'connected' | 'disconnected' | 'connecting'
export type Direction = 'up' | 'down' | 'left' | 'right'

// Authentication types
export interface AuthConfig {
  enabled: boolean
  apiKeys: string[]
  jwt: {
    secret: string
    expiresIn: string
    algorithm: 'HS256' | 'HS384' | 'HS512'
  }
}

export interface JWTPayload {
  sub: string          // Subject (user ID)
  iat: number          // Issued at
  exp: number          // Expiration time
  scope: string[]      // Permissions/scopes
}

export interface AuthRequest {
  apiKey?: string      // API key from header or query
  token?: string       // JWT token from Authorization header
  user?: {             // Authenticated user info
    id: string
    scopes: string[]
  }
}

export interface AuthState {
  isAuthenticated: boolean
  user: {
    id: string
    scopes: string[]
  } | null
  token: string | null
  apiKey: string | null
  isLoading: boolean
  error: string | null
}

export interface LoginCredentials {
  apiKey: string
  userId: string
  scopes?: string[]
}

export interface AuthContextType extends AuthState {
  login: (credentials: LoginCredentials) => Promise<void>
  logout: () => void
  refreshToken: () => Promise<void>
  clearError: () => void
}