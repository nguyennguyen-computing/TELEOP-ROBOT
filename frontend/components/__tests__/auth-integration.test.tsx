/**
 * Tests for authentication integration
 */

import { render, screen, fireEvent, waitFor, act } from '@testing-library/react'
import { AuthProvider, useAuth } from '../auth-provider'
import { AuthGuard } from '../auth-guard'
import { env } from '@/lib/env'

// Mock environment
jest.mock('@/lib/env', () => ({
  env: {
    AUTH_ENABLED: true,
    API_URL: 'http://localhost:3001',
    WS_URL: 'ws://localhost:3001',
    JWT_STORAGE_KEY: 'test_jwt_token',
    API_KEY_STORAGE_KEY: 'test_api_key',
  }
}))

// Mock fetch
global.fetch = jest.fn()

// Mock localStorage
const mockLocalStorage = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
}
Object.defineProperty(window, 'localStorage', {
  value: mockLocalStorage,
})

// Test component that uses auth
function TestComponent() {
  const { isAuthenticated, user, login, logout, isLoading, error } = useAuth()

  if (isLoading) return <div>Loading...</div>
  if (error) return <div>Error: {error}</div>

  return (
    <div>
      <div data-testid="auth-status">
        {isAuthenticated ? 'Authenticated' : 'Not Authenticated'}
      </div>
      {user && (
        <div data-testid="user-info">
          User: {user.id}, Scopes: {user.scopes.join(', ')}
        </div>
      )}
      <button onClick={() => login({ apiKey: 'test-key' })}>
        Login with API Key
      </button>
      <button onClick={logout}>Logout</button>
    </div>
  )
}

describe('Authentication Integration', () => {
  beforeEach(() => {
    jest.clearAllMocks()
    mockLocalStorage.getItem.mockReturnValue(null)
  })

  it('should render login form when not authenticated', async () => {
    await act(async () => {
      render(
        <AuthProvider>
          <AuthGuard>
            <div>Protected Content</div>
          </AuthGuard>
        </AuthProvider>
      )
    })

    await waitFor(() => {
      expect(screen.getByText('Authentication Required')).toBeInTheDocument()
    })
    
    expect(screen.getByLabelText('API Key')).toBeInTheDocument()
  })

  it('should show loading state during initialization', () => {
    render(
      <AuthProvider>
        <TestComponent />
      </AuthProvider>
    )

    expect(screen.getByText('Loading...')).toBeInTheDocument()
  })

  it('should handle API key authentication', async () => {
    const mockFetch = fetch as jest.MockedFunction<typeof fetch>
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ valid: true, user: { id: 'test-user', scopes: ['robot:control'] } }),
    } as Response)

    await act(async () => {
      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      )
    })

    await waitFor(() => {
      expect(screen.getByText('Not Authenticated')).toBeInTheDocument()
    })

    await act(async () => {
      fireEvent.click(screen.getByText('Login with API Key'))
    })

    await waitFor(() => {
      expect(screen.getByText('Authenticated')).toBeInTheDocument()
      expect(screen.getByText('User: test-user, Scopes: robot:control')).toBeInTheDocument()
    })

    expect(mockLocalStorage.setItem).toHaveBeenCalledWith('test_api_key', 'test-key')
  })

  it('should handle authentication errors', async () => {
    const mockFetch = fetch as jest.MockedFunction<typeof fetch>
    mockFetch.mockResolvedValueOnce({
      ok: false,
      json: async () => ({ message: 'Invalid API key' }),
    } as Response)

    await act(async () => {
      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      )
    })

    await waitFor(() => {
      expect(screen.getByText('Not Authenticated')).toBeInTheDocument()
    })

    await act(async () => {
      fireEvent.click(screen.getByText('Login with API Key'))
    })

    await waitFor(() => {
      expect(screen.getByText('Error: Invalid API key')).toBeInTheDocument()
    })
  })

  it('should handle logout', async () => {
    // Start with authenticated state
    mockLocalStorage.getItem.mockReturnValue('test-api-key')
    const mockFetch = fetch as jest.MockedFunction<typeof fetch>
    mockFetch.mockResolvedValueOnce({
      ok: true,
      json: async () => ({ valid: true, user: { id: 'test-user', scopes: ['robot:control'] } }),
    } as Response)

    await act(async () => {
      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      )
    })

    await waitFor(() => {
      expect(screen.getByText('Authenticated')).toBeInTheDocument()
    })

    await act(async () => {
      fireEvent.click(screen.getByText('Logout'))
    })

    await waitFor(() => {
      expect(screen.getByText('Not Authenticated')).toBeInTheDocument()
    })

    expect(mockLocalStorage.removeItem).toHaveBeenCalled()
  })

  it('should protect content with required scopes', async () => {
    await act(async () => {
      render(
        <AuthProvider>
          <AuthGuard requiredScopes={['robot:control']} showLogin={false}>
            <div>Protected Content</div>
          </AuthGuard>
        </AuthProvider>
      )
    })

    await waitFor(() => {
      expect(screen.getByText('Authentication Required')).toBeInTheDocument()
    })
    
    expect(screen.queryByText('Protected Content')).not.toBeInTheDocument()
  })

  it('should bypass authentication when disabled', () => {
    // Mock auth disabled
    jest.mocked(env).AUTH_ENABLED = false

    render(
      <AuthProvider>
        <AuthGuard>
          <div>Protected Content</div>
        </AuthGuard>
      </AuthProvider>
    )

    expect(screen.getByText('Protected Content')).toBeInTheDocument()
  })
})