/**
 * Hook for accessing authenticated API clients
 */

import { useMemo } from 'react'
import { useAuth } from '@/components/auth-provider'
import { createAuthenticatedApiClient, createAuthenticatedWebSocketClient } from '@/lib/api-client'

export function useApiClient() {
  const { token, apiKey } = useAuth()
  
  return useMemo(() => {
    return createAuthenticatedApiClient(token, apiKey)
  }, [token, apiKey])
}

export function useWebSocketClient() {
  const { token, apiKey } = useAuth()
  
  return useMemo(() => {
    return createAuthenticatedWebSocketClient(token, apiKey)
  }, [token, apiKey])
}