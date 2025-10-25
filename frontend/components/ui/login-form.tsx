'use client'

/**
 * Login form component for authentication
 */

import React, { useState } from 'react'
import { Button } from './button'
import { useAuth } from '../auth-provider'
import { LoginCredentials } from '@/types/robot'
import { mockCredentials } from '@/lib/mock-auth'
import { env } from '@/lib/env'

interface LoginFormProps {
  onSuccess?: () => void
  className?: string
}

export function LoginForm({ onSuccess, className = '' }: LoginFormProps) {
  const { login, isLoading, error, clearError } = useAuth()
  const [formData, setFormData] = useState({
    apiKey: '',
    userId: '',
    scopes: ['robot:read', 'robot:control'],
  })

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    clearError()

    try {
      if (!formData.apiKey.trim() || !formData.userId.trim()) {
        throw new Error('API key and User ID are required')
      }

      const credentials: LoginCredentials = {
        apiKey: formData.apiKey.trim(),
        userId: formData.userId.trim(),
        scopes: formData.scopes,
      }

      await login(credentials)
      onSuccess?.()
    } catch (error) {
      // Error is handled by the auth context
      console.error('Login failed:', error)
    }
  }

  const handleInputChange = (field: keyof typeof formData) => (
    e: React.ChangeEvent<HTMLInputElement>
  ) => {
    setFormData(prev => ({
      ...prev,
      [field]: e.target.value,
    }))
  }

  return (
    <div className={`w-full max-w-md mx-auto ${className}`}>
      <div className="bg-card border border-border rounded-lg p-6 shadow-lg">
        <div className="text-center mb-6">
          <h2 className="text-2xl font-bold text-foreground mb-2">
            Authentication Required
          </h2>
          <p className="text-muted-foreground">
            Please authenticate to access the robot control interface
          </p>
        </div>

        <form onSubmit={handleSubmit} className="space-y-4">
          <div>
            <label htmlFor="apiKey" className="block text-sm font-medium text-foreground mb-2">
              API Key
            </label>
            <input
              id="apiKey"
              type="password"
              value={formData.apiKey}
              onChange={handleInputChange('apiKey')}
              placeholder="Enter your API key"
              className="w-full px-3 py-2 border border-input bg-background rounded-md text-foreground placeholder:text-muted-foreground focus:outline-none focus:ring-2 focus:ring-ring focus:border-transparent"
              disabled={isLoading}
              autoComplete="current-password"
            />
          </div>
          
          <div>
            <label htmlFor="userId" className="block text-sm font-medium text-foreground mb-2">
              User ID
            </label>
            <input
              id="userId"
              type="text"
              value={formData.userId}
              onChange={handleInputChange('userId')}
              placeholder="Enter your user ID"
              className="w-full px-3 py-2 border border-input bg-background rounded-md text-foreground placeholder:text-muted-foreground focus:outline-none focus:ring-2 focus:ring-ring focus:border-transparent"
              disabled={isLoading}
              autoComplete="username"
            />
          </div>

          {error && (
            <div className="p-3 bg-destructive/10 border border-destructive/20 rounded-md">
              <p className="text-sm text-destructive">{error}</p>
            </div>
          )}

          <Button
            type="submit"
            className="w-full"
            disabled={isLoading}
          >
            {isLoading ? 'Authenticating...' : 'Login'}
          </Button>
        </form>

        <div className="mt-6 text-center">
          {env.isDevelopment ? (
            <div className="text-xs text-muted-foreground space-y-2">
              <p className="font-medium">Development Mode - Test Credentials:</p>
              <div className="bg-muted/50 rounded p-2 text-left">
                <p className="font-medium mb-1">Test Credentials:</p>
                {mockCredentials.examples.map((example, index) => (
                  <div key={index} className="font-mono text-xs mb-2">
                    <p>API Key: {example.apiKey}</p>
                    <p>User ID: {example.userId}</p>
                    <p>Scopes: {example.scopes.join(', ')}</p>
                  </div>
                ))}
              </div>
            </div>
          ) : (
            <p className="text-xs text-muted-foreground">
              Contact your system administrator for authentication credentials
            </p>
          )}
        </div>
      </div>
    </div>
  )
}