'use client'

/**
 * User profile component showing authentication status and logout option
 */

import React, { useState } from 'react'
import { useAuth } from '../auth-provider'
import { Button } from './button'
import { env } from '@/lib/env'

export function UserProfile() {
  const { user, isAuthenticated, logout, token, apiKey } = useAuth()
  const [isLoggingOut, setIsLoggingOut] = useState(false)

  // Don't render if auth is disabled
  if (!env.AUTH_ENABLED) {
    return null
  }

  // Don't render if not authenticated
  if (!isAuthenticated || !user) {
    return null
  }

  const handleLogout = async () => {
    setIsLoggingOut(true)
    try {
      await logout()
    } catch (error) {
      console.error('Logout failed:', error)
    } finally {
      setIsLoggingOut(false)
    }
  }

  const authMethod = token ? 'JWT Token' : apiKey ? 'API Key' : 'Unknown'

  return (
    <div className="bg-card border border-border rounded-lg p-4 shadow-sm">
      <div className="flex items-center justify-between">
        <div className="flex-1">
          <div className="flex items-center space-x-3">
            <div className="w-8 h-8 bg-primary rounded-full flex items-center justify-center">
              <span className="text-primary-foreground text-sm font-medium">
                {user.id.charAt(0).toUpperCase()}
              </span>
            </div>
            <div>
              <p className="text-sm font-medium text-foreground">
                {user.id}
              </p>
              <p className="text-xs text-muted-foreground">
                {authMethod} • {user.scopes.length} permission{user.scopes.length !== 1 ? 's' : ''}
              </p>
            </div>
          </div>
          
          {user.scopes.length > 0 && (
            <div className="mt-2">
              <p className="text-xs text-muted-foreground mb-1">Permissions:</p>
              <div className="flex flex-wrap gap-1">
                {user.scopes.map((scope) => (
                  <span
                    key={scope}
                    className="inline-flex items-center px-2 py-1 rounded-md text-xs font-medium bg-secondary text-secondary-foreground"
                  >
                    {scope}
                  </span>
                ))}
              </div>
            </div>
          )}
        </div>

        <Button
          variant="outline"
          size="sm"
          onClick={handleLogout}
          disabled={isLoggingOut}
          className="ml-4"
        >
          {isLoggingOut ? 'Logging out...' : 'Logout'}
        </Button>
      </div>
    </div>
  )
}