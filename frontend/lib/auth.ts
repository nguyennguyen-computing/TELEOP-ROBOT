/**
 * Authentication utilities for token management and API communication
 */

import { env } from './env'
import { JWTPayload, LoginCredentials, AuthState } from '@/types/robot'
import { MockAuthAPI } from './mock-auth'

// Token storage utilities
export const tokenStorage = {
  getToken(): string | null {
    if (typeof window === 'undefined') return null
    return localStorage.getItem(env.JWT_STORAGE_KEY)
  },

  setToken(token: string): void {
    if (typeof window === 'undefined') return
    localStorage.setItem(env.JWT_STORAGE_KEY, token)
  },

  removeToken(): void {
    if (typeof window === 'undefined') return
    localStorage.removeItem(env.JWT_STORAGE_KEY)
  },

  getApiKey(): string | null {
    if (typeof window === 'undefined') return null
    return localStorage.getItem(env.API_KEY_STORAGE_KEY)
  },

  setApiKey(apiKey: string): void {
    if (typeof window === 'undefined') return
    localStorage.setItem(env.API_KEY_STORAGE_KEY, apiKey)
  },

  removeApiKey(): void {
    if (typeof window === 'undefined') return
    localStorage.removeItem(env.API_KEY_STORAGE_KEY)
  },

  clear(): void {
    if (typeof window === 'undefined') return
    localStorage.removeItem(env.JWT_STORAGE_KEY)
    localStorage.removeItem(env.API_KEY_STORAGE_KEY)
  }
}

// JWT utilities
export const jwtUtils = {
  decode(token: string): JWTPayload | null {
    try {
      const payload = token.split('.')[1]
      if (!payload) return null

      const decoded = JSON.parse(atob(payload))
      return decoded as JWTPayload
    } catch (error) {
      console.error('Failed to decode JWT:', error)
      return null
    }
  },

  isExpired(token: string): boolean {
    const payload = this.decode(token)
    if (!payload) return true

    const now = Math.floor(Date.now() / 1000)
    return payload.exp < now
  },

  isValid(token: string): boolean {
    const payload = this.decode(token)
    return payload !== null && !this.isExpired(token)
  },

  getExpirationTime(token: string): Date | null {
    const payload = this.decode(token)
    if (!payload) return null

    return new Date(payload.exp * 1000)
  }
}

// Authentication API client
export class AuthAPI {
  private baseUrl: string
  private mockAPI: MockAuthAPI

  constructor(baseUrl: string = env.API_URL) {
    this.baseUrl = baseUrl
    this.mockAPI = new MockAuthAPI()
  }

  private async tryMockFallback<T>(
    realApiCall: () => Promise<T>,
    mockApiCall: () => Promise<T>
  ): Promise<T> {
    if (env.isDevelopment) {
      try {
        return await realApiCall()
      } catch (error) {
        console.warn('Real API failed, falling back to mock:', error)
        return await mockApiCall()
      }
    }
    return await realApiCall()
  }

  async login(credentials: LoginCredentials): Promise<{ token?: string; user?: any }> {
    return this.tryMockFallback(
      async () => {
        const { apiKey, userId, scopes = ['robot:read', 'robot:control'] } = credentials

        const response = await fetch(`${this.baseUrl}/api/auth/token`, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
            'Authorization': `ApiKey ${apiKey}`,
          },
          body: JSON.stringify({ userId, scopes }),
        })

        if (!response.ok) {
          const error = await response.json().catch(() => ({ message: 'Login failed' }))
          throw new Error(error.error || error.message || 'Authentication failed')
        }

        const result = await response.json()
        return {
          token: result.data?.token,
          user: {
            id: result.data?.userId || userId,
            scopes: result.data?.scopes || scopes
          }
        }
      },
      async () => {
        const mockResult = await this.mockAPI.login(credentials)
        return {
          token: mockResult.token,
          user: mockResult.user
        }
      }
    )
  }

  async verifyAuth(token: string): Promise<{ authenticated: boolean; user?: any }> {
    return this.tryMockFallback(
      async () => {
        const response = await fetch(`${this.baseUrl}/api/auth/verify`, {
          method: 'GET',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        })

        if (!response.ok) {
          return { authenticated: false }
        }

        const result = await response.json()
        return {
          authenticated: result.data?.authenticated || false,
          user: result.data?.auth?.user
        }
      },
      () => Promise.resolve({ authenticated: true, user: { id: 'mock-user', scopes: ['robot:control', 'robot:read'] } })
    )
  }

  async logout(): Promise<void> {
    // Backend doesn't have logout endpoint, just clear local storage
    console.log('Logging out (no backend endpoint)')
  }
}

// Create default auth API instance
export const authAPI = new AuthAPI()

// Authentication state management utilities
export const authUtils = {
  createInitialState(): AuthState {
    return {
      isAuthenticated: false,
      user: null,
      token: null,
      apiKey: null,
      isLoading: false,
      error: null,
    }
  },

  async initializeAuth(): Promise<Partial<AuthState>> {
    if (!env.AUTH_ENABLED) {
      return {
        isAuthenticated: true,
        user: { id: 'anonymous', scopes: ['robot:control', 'robot:read'] },
        isLoading: false,
      }
    }

    const token = tokenStorage.getToken()
    const apiKey = tokenStorage.getApiKey()

    // Try JWT token first
    if (token && jwtUtils.isValid(token)) {
      try {
        const result = await authAPI.verifyAuth(token)
        if (result.authenticated) {
          const payload = jwtUtils.decode(token)
          return {
            isAuthenticated: true,
            user: result.user || (payload ? { id: payload.sub, scopes: payload.scope } : { id: 'unknown', scopes: [] }),
            token,
            isLoading: false,
          }
        }
      } catch (error) {
        console.error('Token verification failed:', error)
        tokenStorage.removeToken()
      }
    }

    // If we have API key stored, we need to generate a new token
    if (apiKey) {
      try {
        // We can't validate API key directly, but we can try to generate a token
        // This requires userId, so we'll skip this for now
        console.log('API key found but no way to validate without userId')
        tokenStorage.clear()
      } catch (error) {
        console.error('API key handling failed:', error)
        tokenStorage.clear()
      }
    }

    // No valid authentication found
    tokenStorage.clear()
    return {
      isAuthenticated: false,
      user: null,
      token: null,
      apiKey: null,
      isLoading: false,
    }
  },

  getAuthHeaders(token?: string | null, apiKey?: string | null): Record<string, string> {
    const headers: Record<string, string> = {}

    if (token) {
      headers['Authorization'] = `Bearer ${token}`
    } else if (apiKey) {
      headers['Authorization'] = `ApiKey ${apiKey}`
    }

    return headers
  },
}