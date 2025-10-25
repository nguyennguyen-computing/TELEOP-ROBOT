'use client'

/**
 * Authentication context provider for managing user authentication state
 */

import React, { createContext, useContext, useReducer, useEffect, useCallback } from 'react'
import { AuthContextType, AuthState, LoginCredentials } from '@/types/robot'
import { authAPI, authUtils, tokenStorage, jwtUtils } from '@/lib/auth'
import { env } from '@/lib/env'

// Auth actions
type AuthAction =
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: string | null }
  | { type: 'LOGIN_SUCCESS'; payload: { user: any; token?: string; apiKey?: string } }
  | { type: 'LOGOUT' }
  | { type: 'REFRESH_TOKEN_SUCCESS'; payload: { user: any; token: string } }
  | { type: 'INITIALIZE_SUCCESS'; payload: Partial<AuthState> }

// Auth reducer
function authReducer(state: AuthState, action: AuthAction): AuthState {
  switch (action.type) {
    case 'SET_LOADING':
      return { ...state, isLoading: action.payload }
    
    case 'SET_ERROR':
      return { ...state, error: action.payload, isLoading: false }
    
    case 'LOGIN_SUCCESS':
      return {
        ...state,
        isAuthenticated: true,
        user: action.payload.user,
        token: action.payload.token || null,
        apiKey: action.payload.apiKey || null,
        error: null,
        isLoading: false,
      }
    
    case 'LOGOUT':
      return {
        ...authUtils.createInitialState(),
        isAuthenticated: false,
      }
    
    case 'REFRESH_TOKEN_SUCCESS':
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token,
        error: null,
      }
    
    case 'INITIALIZE_SUCCESS':
      return {
        ...state,
        ...action.payload,
      }
    
    default:
      return state
  }
}

// Create context
const AuthContext = createContext<AuthContextType | null>(null)

// Auth provider component
export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [state, dispatch] = useReducer(authReducer, authUtils.createInitialState())

  // Initialize authentication on mount
  useEffect(() => {
    const initAuth = async () => {
      dispatch({ type: 'SET_LOADING', payload: true })
      
      try {
        const authState = await authUtils.initializeAuth()
        dispatch({ type: 'INITIALIZE_SUCCESS', payload: authState })
      } catch (error) {
        console.error('Auth initialization failed:', error)
        dispatch({ type: 'SET_ERROR', payload: 'Failed to initialize authentication' })
      }
    }

    initAuth()
  }, [])

  // Logout function
  const logout = useCallback(async () => {
    try {
      await authAPI.logout()
    } catch (error) {
      console.error('Logout error:', error)
    } finally {
      tokenStorage.clear()
      dispatch({ type: 'LOGOUT' })
    }
  }, [])

  // Auto-logout when token expires (no refresh endpoint available)
  useEffect(() => {
    if (!state.token || !state.isAuthenticated) return

    const checkTokenExpiration = () => {
      if (jwtUtils.isExpired(state.token!)) {
        console.log('Token expired, logging out')
        logout()
      }
    }

    // Check every minute
    const interval = setInterval(checkTokenExpiration, 60000)
    
    // Check immediately
    checkTokenExpiration()

    return () => clearInterval(interval)
  }, [state.token, state.isAuthenticated, logout])

  // Login function
  const login = useCallback(async (credentials: LoginCredentials) => {
    dispatch({ type: 'SET_LOADING', payload: true })
    dispatch({ type: 'SET_ERROR', payload: null })

    try {
      // Handle API key + userId authentication
      if (credentials.apiKey && credentials.userId) {
        const result = await authAPI.login(credentials)
        if (result.token) {
          const payload = jwtUtils.decode(result.token)
          if (payload) {
            tokenStorage.setToken(result.token)
            tokenStorage.setApiKey(credentials.apiKey)
            dispatch({
              type: 'LOGIN_SUCCESS',
              payload: {
                user: result.user || { id: payload.sub, scopes: payload.scope },
                token: result.token,
                apiKey: credentials.apiKey,
              },
            })
            return
          }
        }
      }

      throw new Error('API key and user ID are required')
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Login failed'
      dispatch({ type: 'SET_ERROR', payload: message })
      throw error
    }
  }, [])



  // Refresh token function (not available in backend, just logout)
  const refreshToken = useCallback(async () => {
    console.log('Refresh token not available, user needs to login again')
    logout()
  }, [logout])

  // Clear error function
  const clearError = useCallback(() => {
    dispatch({ type: 'SET_ERROR', payload: null })
  }, [])

  const contextValue: AuthContextType = {
    ...state,
    login,
    logout,
    refreshToken,
    clearError,
  }

  return (
    <AuthContext.Provider value={contextValue}>
      {children}
    </AuthContext.Provider>
  )
}

// Hook to use auth context
export function useAuth(): AuthContextType {
  const context = useContext(AuthContext)
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider')
  }
  return context
}

// Hook to check if user has required scopes
export function useAuthScopes(requiredScopes: string[] = []): boolean {
  const { user, isAuthenticated } = useAuth()
  
  if (!env.AUTH_ENABLED) return true
  if (!isAuthenticated || !user) return false
  
  return requiredScopes.every(scope => user.scopes.includes(scope))
}