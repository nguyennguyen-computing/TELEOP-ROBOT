'use client'

/**
 * Authentication guard component that protects routes and components
 */

import React from 'react'
import { useAuth, useAuthScopes } from './auth-provider'
import { LoginForm } from './ui/login-form'
import { env } from '@/lib/env'

interface AuthGuardProps {
  children: React.ReactNode
  requiredScopes?: string[]
  fallback?: React.ReactNode
  showLogin?: boolean
}

export function AuthGuard({
  children,
  requiredScopes = [],
  fallback,
  showLogin = true
}: AuthGuardProps) {
  const { isAuthenticated, isLoading, user } = useAuth()
  const hasRequiredScopes = useAuthScopes(requiredScopes)

  // If authentication is disabled, always render children
  if (!env.AUTH_ENABLED) {
    return <>{children}</>
  }

  // Show loading state
  if (isLoading) {
    return (
      <div className="flex items-center justify-center min-h-screen">
        <div className="text-center">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-primary mx-auto mb-4"></div>
          <p className="text-muted-foreground">Initializing authentication...</p>
        </div>
      </div>
    )
  }

  // Show login form if not authenticated
  if (!isAuthenticated) {
    if (showLogin) {
      return (
        <div className="flex items-center justify-center min-h-screen p-4">
          <LoginForm />
        </div>
      )
    }

    return fallback || (
      <div className="flex items-center justify-center min-h-screen">
        <div className="text-center">
          <h2 className="text-2xl font-bold text-foreground mb-2">
            Authentication Required
          </h2>
          <p className="text-muted-foreground">
            Please authenticate to access this resource
          </p>
        </div>
      </div>
    )
  }

  // Check required scopes
  if (requiredScopes.length > 0 && !hasRequiredScopes) {
    return fallback || (
      <div className="flex items-center justify-center min-h-screen">
        <div className="text-center">
          <h2 className="text-2xl font-bold text-foreground mb-2">
            Access Denied
          </h2>
          <p className="text-muted-foreground mb-4">
            You don&apos;t have permission to access this resource
          </p>
          <div className="text-sm text-muted-foreground">
            <p>Required scopes: {requiredScopes.join(', ')}</p>
            <p>Your scopes: {user?.scopes.join(', ') || 'None'}</p>
          </div>
        </div>
      </div>
    )
  }

  // Render protected content
  return <>{children}</>
}

// Higher-order component for protecting pages
export function withAuthGuard<P extends object>(
  Component: React.ComponentType<P>,
  options: Omit<AuthGuardProps, 'children'> = {}
) {
  return function AuthGuardedComponent(props: P) {
    return (
      <AuthGuard {...options}>
        <Component {...props} />
      </AuthGuard>
    )
  }
}