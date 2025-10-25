/**
 * Authenticated API client for making requests to the backend
 */

import { env } from './env'
import { authUtils } from './auth'
import { VelocityCommand } from '@/types/robot'

export interface ApiResponse<T = any> {
  ok: boolean
  data?: T
  error?: string
  status: number
}

export class ApiClient {
  private baseUrl: string
  private getAuthHeaders: () => Record<string, string>

  constructor(
    baseUrl: string = env.API_URL,
    getAuthHeaders: () => Record<string, string> = () => ({})
  ) {
    this.baseUrl = baseUrl
    this.getAuthHeaders = getAuthHeaders
  }

  private async request<T = any>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<ApiResponse<T>> {
    const url = `${this.baseUrl}${endpoint}`
    const authHeaders = this.getAuthHeaders()

    const config: RequestInit = {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        ...authHeaders,
        ...options.headers,
      },
    }

    try {
      const response = await fetch(url, config)

      let data: any = null
      const contentType = response.headers.get('content-type')

      if (contentType && contentType.includes('application/json')) {
        data = await response.json()
      } else {
        const text = await response.text()
        if (text) {
          try {
            data = JSON.parse(text)
          } catch {
            data = { message: text }
          }
        }
      }

      return {
        ok: response.ok,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data?.message || data?.error || `HTTP ${response.status}`),
        status: response.status,
      }
    } catch (error) {
      return {
        ok: false,
        error: error instanceof Error ? error.message : 'Network error',
        status: 0,
      }
    }
  }

  // Velocity command endpoints
  async sendVelocityCommand(command: VelocityCommand): Promise<ApiResponse> {
    return this.request('/api/vel', {
      method: 'POST',
      body: JSON.stringify(command),
    })
  }

  // Logs endpoints
  async getLogs(limit?: number): Promise<ApiResponse<any[]>> {
    const params = new URLSearchParams()
    if (limit) {
      params.append('limit', limit.toString())
    }

    const endpoint = `/api/logs${params.toString() ? `?${params.toString()}` : ''}`
    return this.request(endpoint)
  }

  // Health check
  async healthCheck(): Promise<ApiResponse> {
    return this.request('/api/health')
  }

  // Authentication endpoints
  async generateToken(credentials: { userId: string; scopes?: string[] }, apiKey: string): Promise<ApiResponse> {
    return this.request('/api/auth/token', {
      method: 'POST',
      headers: {
        'Authorization': `ApiKey ${apiKey}`,
      },
      body: JSON.stringify(credentials),
    })
  }

  async verifyAuth(): Promise<ApiResponse> {
    return this.request('/api/auth/verify', {
      method: 'GET',
    })
  }

  // WebSocket and Fleet endpoints
  async getWebSocketStats(): Promise<ApiResponse> {
    return this.request('/api/websocket/stats')
  }

  async getFleetStats(): Promise<ApiResponse> {
    return this.request('/api/fleet/stats')
  }

  async getFleetHealth(): Promise<ApiResponse> {
    return this.request('/api/fleet/health')
  }
}

// Create authenticated API client factory
export function createAuthenticatedApiClient(
  token?: string | null,
  apiKey?: string | null
): ApiClient {
  return new ApiClient(env.API_URL, () => authUtils.getAuthHeaders(token, apiKey))
}

// Default API client (will need to be updated with auth context)
export const apiClient = new ApiClient()

// WebSocket client with authentication and enhanced real-time communication
export class AuthenticatedWebSocketClient {
  private ws: WebSocket | null = null
  private url: string
  private getAuthHeaders: () => Record<string, string>
  private reconnectAttempts = 0
  private maxReconnectAttempts = 5
  private reconnectDelay = 1000
  private listeners: Map<string, Set<(data: any) => void>> = new Map()
  private heartbeatInterval: NodeJS.Timeout | null = null
  private heartbeatTimeout: NodeJS.Timeout | null = null
  private lastPongTime: number = 0
  private connectionStartTime: number = 0
  private isManualDisconnect = false
  
  // Enhanced connection stability features
  private connectionQuality: 'excellent' | 'good' | 'poor' | 'bad' = 'excellent'
  private consecutiveFailures = 0
  private lastSuccessfulConnection = 0
  private visibilityChangeHandler: (() => void) | null = null
  private networkChangeHandler: (() => void) | null = null

  constructor(
    url: string = env.WS_URL,
    getAuthHeaders: () => Record<string, string> = () => ({})
  ) {
    this.url = url
    this.getAuthHeaders = getAuthHeaders
    this.setupBrowserEventHandlers()
  }

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        this.isManualDisconnect = false
        this.connectionStartTime = Date.now()

        // Add auth headers as query parameters for WebSocket
        const authHeaders = this.getAuthHeaders()
        const wsUrl = new URL(this.url)

        if (authHeaders.Authorization) {
          const [type, token] = authHeaders.Authorization.split(' ')
          if (type === 'Bearer') {
            wsUrl.searchParams.set('token', token)
          } else if (type === 'ApiKey') {
            wsUrl.searchParams.set('apiKey', token)
          }
        }

        this.ws = new WebSocket(wsUrl.toString())

        // Set a connection timeout
        const connectionTimeout = setTimeout(() => {
          if (this.ws && this.ws.readyState === WebSocket.CONNECTING) {
            this.ws.close()
            reject(new Error('WebSocket connection timeout'))
          }
        }, 5000) // 5 second timeout

        this.ws.onopen = () => {
          clearTimeout(connectionTimeout)
          this.reconnectAttempts = 0
          this.consecutiveFailures = 0
          this.lastSuccessfulConnection = Date.now()
          this.startHeartbeat()

          console.log('WebSocket onopen - connection established', {
            attempts: this.reconnectAttempts,
            connectionTime: Date.now() - this.connectionStartTime,
            quality: this.connectionQuality
          })

          // Don't emit connection_status here - let the backend send it
          // or emit it after a small delay to avoid race conditions
          setTimeout(() => {
            // Only emit if we haven't received a connection_status from backend
            if (this.ws && this.ws.readyState === WebSocket.OPEN) {
              this.emit('connection_status', {
                status: 'connected',
                timestamp: new Date(),
                connectionTime: Date.now() - this.connectionStartTime,
                source: 'client_onopen',
                quality: this.connectionQuality
              })
            }
          }, 100) // Small delay to let backend messages arrive first

          resolve()
        }

        this.ws.onmessage = (event) => {
          try {
            const message = JSON.parse(event.data)
            console.log('WebSocket message received:', message)

            // Handle heartbeat pong messages
            if (message.type === 'pong') {
              this.lastPongTime = Date.now()
              
              // Calculate latency correctly - message.data.timestamp is the original ping timestamp
              const pingTimestamp = message.data?.timestamp || 0
              const latency = this.lastPongTime - pingTimestamp
              
              console.log('Pong received:', {
                pingTimestamp,
                pongTime: this.lastPongTime,
                latency
              })
              
              // Clear the heartbeat timeout since we got a response
              if (this.heartbeatTimeout) {
                clearTimeout(this.heartbeatTimeout)
                this.heartbeatTimeout = null
              }
              
              this.emit('heartbeat', { latency })
              return
            }

            // Handle velocity updates from server
            if (message.type === 'velocity_update') {
              this.emit('velocity_update', {
                vx: message.data.vx,
                vy: message.data.vy,
                levels: message.data.levels,
                timestamp: new Date(message.data.timestamp || message.timestamp)
              })
              return
            }

            // Handle connection status updates from server
            if (message.type === 'connection_status') {
              // Backend sends connection_status messages, but we need to interpret them
              // as connection status updates for the frontend
              const isConnected = message.data.connected
              console.log('Backend connection_status received:', message.data)
              
              this.emit('connection_status', {
                status: isConnected ? 'connected' : 'disconnected',
                timestamp: new Date(),
                connectionId: message.data.connectionId,
                clientCount: message.data.clientCount,
                source: 'backend_message',
                ...message.data
              })
              return
            }

            // Handle ping messages (send pong back)
            if (message.type === 'ping') {
              this.send('pong', { timestamp: message.data?.timestamp || Date.now() })
              return
            }

            // Emit generic message
            this.emit(message.type || 'message', message)
          } catch (error) {
            console.error('Failed to parse WebSocket message:', error)
          }
        }

        this.ws.onclose = (event) => {
          clearTimeout(connectionTimeout)
          this.stopHeartbeat()

          const connectionDuration = this.lastSuccessfulConnection > 0 ? 
            Date.now() - this.lastSuccessfulConnection : 0

          console.log(`WebSocket closed with code: ${event.code}, reason: ${event.reason}`, {
            wasManual: this.isManualDisconnect,
            connectionDuration: this.formatDuration(connectionDuration),
            consecutiveFailures: this.consecutiveFailures
          })

          // Update connection quality based on close reason
          this.updateConnectionQuality(event.code, event.reason)

          // Emit close event to notify components
          this.emit('connection_status', {
            status: 'disconnected',
            code: event.code,
            reason: event.reason,
            timestamp: new Date(),
            wasManual: this.isManualDisconnect,
            connectionDuration,
            quality: this.connectionQuality
          })

          // Only attempt reconnection if it was an unexpected close and not manual
          if (event.code !== 1000 && !this.isManualDisconnect) {
            this.consecutiveFailures++
            this.handleReconnect()
          }
        }

        this.ws.onerror = (error) => {
          clearTimeout(connectionTimeout)
          console.warn('WebSocket connection failed:', error)
          this.emit('connection_status', {
            status: 'disconnected',
            error: 'Connection failed',
            timestamp: new Date()
          })
          reject(new Error('WebSocket connection failed'))
        }
      } catch (error) {
        reject(error)
      }
    })
  }

  private handleReconnect() {
    if (this.reconnectAttempts < this.maxReconnectAttempts && !this.isManualDisconnect) {
      this.reconnectAttempts++
      
      // Adaptive delay based on connection quality and consecutive failures
      let baseDelay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1)
      
      // Increase delay for poor connection quality
      if (this.connectionQuality === 'poor') baseDelay *= 2
      if (this.connectionQuality === 'bad') baseDelay *= 3
      
      // Add jitter to prevent thundering herd
      const jitter = Math.random() * 1000
      const delay = Math.min(baseDelay + jitter, 30000) // Max 30s delay

      console.log(`WebSocket reconnection attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts} in ${Math.round(delay)}ms`, {
        quality: this.connectionQuality,
        consecutiveFailures: this.consecutiveFailures,
        baseDelay: Math.round(baseDelay)
      })

      // Emit reconnecting status
      this.emit('connection_status', {
        status: 'connecting',
        reconnectAttempt: this.reconnectAttempts,
        maxAttempts: this.maxReconnectAttempts,
        delay: Math.round(delay),
        quality: this.connectionQuality,
        timestamp: new Date()
      })

      setTimeout(() => {
        this.connect().catch((error) => {
          console.warn(`WebSocket reconnection attempt ${this.reconnectAttempts} failed:`, error)

          // If this was the last attempt, emit final disconnected status
          if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            this.emit('connection_status', {
              status: 'disconnected',
              error: 'Max reconnection attempts reached',
              finalAttempt: true,
              quality: this.connectionQuality,
              timestamp: new Date()
            })
          }
        })
      }, delay)
    } else {
      console.warn('WebSocket max reconnection attempts reached. Giving up.')
      this.emit('connection_status', {
        status: 'disconnected',
        error: 'Max reconnection attempts reached',
        finalAttempt: true,
        quality: this.connectionQuality,
        timestamp: new Date()
      })
    }
  }

  // Setup browser event handlers for better connection management
  private setupBrowserEventHandlers() {
    // Handle page visibility changes
    this.visibilityChangeHandler = () => {
      if (document.hidden) {
        console.log('Page hidden - pausing heartbeat to save resources')
        this.stopHeartbeat()
      } else {
        console.log('Page visible - resuming heartbeat')
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
          this.startHeartbeat()
        }
      }
    }
    
    document.addEventListener('visibilitychange', this.visibilityChangeHandler)

    // Handle network changes
    if ('connection' in navigator) {
      this.networkChangeHandler = () => {
        const connection = (navigator as any).connection
        console.log('Network changed:', {
          effectiveType: connection.effectiveType,
          downlink: connection.downlink,
          rtt: connection.rtt
        })
        
        // Update connection quality based on network info
        this.updateConnectionQualityFromNetwork(connection)
      }
      
      ;(navigator as any).connection.addEventListener('change', this.networkChangeHandler)
    }

    // Handle online/offline events
    window.addEventListener('online', () => {
      console.log('Network: ONLINE - attempting reconnection if needed')
      if (!this.isConnected && !this.isManualDisconnect) {
        this.resetReconnectionAttempts()
        this.connect().catch(error => {
          console.warn('Failed to reconnect after coming online:', error)
        })
      }
    })

    window.addEventListener('offline', () => {
      console.log('Network: OFFLINE - connection will be restored when online')
      this.connectionQuality = 'bad'
    })
  }

  // Update connection quality based on close code and reason
  private updateConnectionQuality(code: number, reason: string) {
    if (code === 1000) {
      // Normal closure
      this.connectionQuality = 'excellent'
    } else if (code === 1001 || code === 1005) {
      // Going away or no status - might be normal
      this.connectionQuality = 'good'
    } else if (code === 1006) {
      // Abnormal closure - network issues
      this.connectionQuality = this.consecutiveFailures > 2 ? 'bad' : 'poor'
    } else if (reason.includes('timeout')) {
      // Timeout related
      this.connectionQuality = 'poor'
    } else {
      // Other errors
      this.connectionQuality = 'poor'
    }
    
    console.log('Connection quality updated:', this.connectionQuality, {
      code,
      reason,
      consecutiveFailures: this.consecutiveFailures
    })
  }

  // Update connection quality from network information
  private updateConnectionQualityFromNetwork(connection: any) {
    if (connection.effectiveType === '4g' && connection.rtt < 100) {
      this.connectionQuality = 'excellent'
    } else if (connection.effectiveType === '4g' || (connection.effectiveType === '3g' && connection.rtt < 200)) {
      this.connectionQuality = 'good'
    } else if (connection.effectiveType === '3g' || connection.effectiveType === '2g') {
      this.connectionQuality = 'poor'
    } else {
      this.connectionQuality = 'bad'
    }
  }

  // Format duration for logging
  private formatDuration(ms: number): string {
    if (ms < 1000) return `${ms}ms`
    if (ms < 60000) return `${Math.round(ms / 1000)}s`
    if (ms < 3600000) return `${Math.round(ms / 60000)}m`
    return `${Math.round(ms / 3600000)}h`
  }

  private startHeartbeat() {
    this.stopHeartbeat() // Clear any existing heartbeat

    // Adaptive heartbeat interval based on connection quality
    let heartbeatInterval = 30000 // Default 30s
    let heartbeatTimeout = 10000  // Default 10s
    
    switch (this.connectionQuality) {
      case 'excellent':
        heartbeatInterval = 30000 // 30s
        heartbeatTimeout = 10000  // 10s
        break
      case 'good':
        heartbeatInterval = 25000 // 25s - more frequent
        heartbeatTimeout = 12000  // 12s - longer timeout
        break
      case 'poor':
        heartbeatInterval = 20000 // 20s - more frequent
        heartbeatTimeout = 15000  // 15s - longer timeout
        break
      case 'bad':
        heartbeatInterval = 15000 // 15s - very frequent
        heartbeatTimeout = 20000  // 20s - much longer timeout
        break
    }

    console.log('Starting adaptive heartbeat:', {
      quality: this.connectionQuality,
      interval: heartbeatInterval + 'ms',
      timeout: heartbeatTimeout + 'ms'
    })

    // Send ping at adaptive intervals
    this.heartbeatInterval = setInterval(() => {
      // Skip heartbeat if page is hidden to save resources
      if (document.hidden) {
        console.log('Heartbeat skipped - page hidden')
        return
      }
      
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        const timestamp = Date.now()
        console.log('Sending heartbeat ping:', timestamp, `(quality: ${this.connectionQuality})`)
        
        const success = this.send('ping', { timestamp })
        
        if (success) {
          // Set adaptive timeout for pong response
          this.heartbeatTimeout = setTimeout(() => {
            console.warn(`💔 WebSocket heartbeat timeout - no pong received within ${heartbeatTimeout}ms`, {
              quality: this.connectionQuality,
              consecutiveFailures: this.consecutiveFailures
            })
            if (this.ws) {
              this.ws.close(1000, 'Heartbeat timeout')
            }
          }, heartbeatTimeout)
        } else {
          console.warn('💔 Failed to send heartbeat ping - WebSocket not ready')
        }
      } else {
        console.warn('💔 Heartbeat skipped - WebSocket not connected')
      }
    }, heartbeatInterval)
  }

  private stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval)
      this.heartbeatInterval = null
    }
    if (this.heartbeatTimeout) {
      clearTimeout(this.heartbeatTimeout)
      this.heartbeatTimeout = null
    }
  }

  send(type: string, data: any) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const message = {
        type,
        data,
        timestamp: new Date().toISOString()
      }
      console.log('Sending WebSocket message:', message)
      this.ws.send(JSON.stringify(message))
      return true
    }
    return false
  }

  on(event: string, callback: (data: any) => void) {
    if (!this.listeners.has(event)) {
      this.listeners.set(event, new Set())
    }
    const listeners = this.listeners.get(event)!

    // Check if this callback is already registered to avoid duplicates
    if (!listeners.has(callback)) {
      listeners.add(callback)
      console.log(`Added listener for '${event}'. Total listeners: ${listeners.size}`)
    } else {
      console.warn(`⚠️ Listener for '${event}' already exists, skipping duplicate`)
    }
  }

  off(event: string, callback: (data: any) => void) {
    const eventListeners = this.listeners.get(event)
    if (eventListeners) {
      const removed = eventListeners.delete(callback)
      if (removed) {
        console.log(`Removed listener for '${event}'. Remaining listeners: ${eventListeners.size}`)
      } else {
        console.warn(`⚠️ Tried to remove non-existent listener for '${event}'`)
      }
    }
  }

  // Method to clear all listeners for an event
  clearListeners(event?: string) {
    if (event) {
      const listeners = this.listeners.get(event)
      if (listeners) {
        listeners.clear()
        console.log(`Cleared all listeners for '${event}'`)
      }
    } else {
      this.listeners.clear()
      console.log('Cleared all event listeners')
    }
  }

  private emit(event: string, data: any) {
    const eventListeners = this.listeners.get(event)
    const listenerCount = eventListeners?.size || 0

    console.log(`Emitting event '${event}' to ${listenerCount} listeners`)

    if (eventListeners && listenerCount > 0) {
      // Convert Set to Array to avoid issues with concurrent modifications
      const listenersArray = Array.from(eventListeners)

      listenersArray.forEach((callback, index) => {
        try {
          console.log(`Calling listener ${index + 1}/${listenerCount} for '${event}'`)
          callback(data)
        } catch (error) {
          console.error(`❌ Error in listener ${index + 1} for '${event}':`, error)
        }
      })
    } else {
      console.log(`No listeners registered for event '${event}'`)
    }
  }

  disconnect() {
    this.isManualDisconnect = true
    this.stopHeartbeat()

    if (this.ws) {
      this.ws.close(1000, 'Manual disconnect')
      this.ws = null
    }

    // Clear reconnection attempts
    this.reconnectAttempts = 0
    this.consecutiveFailures = 0
    
    // Cleanup browser event handlers
    this.cleanupBrowserEventHandlers()
  }

  // Cleanup browser event handlers
  private cleanupBrowserEventHandlers() {
    if (this.visibilityChangeHandler) {
      document.removeEventListener('visibilitychange', this.visibilityChangeHandler)
      this.visibilityChangeHandler = null
    }
    
    if (this.networkChangeHandler && 'connection' in navigator) {
      ;(navigator as any).connection.removeEventListener('change', this.networkChangeHandler)
      this.networkChangeHandler = null
    }
  }

  get isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN
  }

  get webSocket(): WebSocket | null {
    return this.ws
  }

  get connectionUrl(): string {
    return this.url
  }

  get connectionState(): {
    readyState: number | null
    reconnectAttempts: number
    maxReconnectAttempts: number
    lastPongTime: number
    isManualDisconnect: boolean
  } {
    return {
      readyState: this.ws?.readyState || null,
      reconnectAttempts: this.reconnectAttempts,
      maxReconnectAttempts: this.maxReconnectAttempts,
      lastPongTime: this.lastPongTime,
      isManualDisconnect: this.isManualDisconnect
    }
  }

  // Method to reset reconnection attempts (useful for manual reconnection)
  resetReconnectionAttempts() {
    this.reconnectAttempts = 0
  }

  // Method to reset the entire client state
  reset() {
    console.log('Resetting WebSocket client state...')
    this.disconnect()
    this.clearListeners()
    this.reconnectAttempts = 0
    this.lastPongTime = 0
    this.connectionStartTime = 0
    this.isManualDisconnect = false
    this.consecutiveFailures = 0
    this.lastSuccessfulConnection = 0
    this.connectionQuality = 'excellent'
  }

  // Get connection diagnostics
  getConnectionDiagnostics() {
    const now = Date.now()
    return {
      isConnected: this.isConnected,
      connectionQuality: this.connectionQuality,
      reconnectAttempts: this.reconnectAttempts,
      consecutiveFailures: this.consecutiveFailures,
      lastSuccessfulConnection: this.lastSuccessfulConnection,
      connectionDuration: this.lastSuccessfulConnection > 0 ? now - this.lastSuccessfulConnection : 0,
      lastPongTime: this.lastPongTime,
      timeSinceLastPong: this.lastPongTime > 0 ? now - this.lastPongTime : 0,
      readyState: this.ws?.readyState || null,
      url: this.url,
      isManualDisconnect: this.isManualDisconnect
    }
  }
}

// Create authenticated WebSocket client factory
export function createAuthenticatedWebSocketClient(
  token?: string | null,
  apiKey?: string | null
): AuthenticatedWebSocketClient {
  return new AuthenticatedWebSocketClient(env.WS_URL, () => authUtils.getAuthHeaders(token, apiKey))
}