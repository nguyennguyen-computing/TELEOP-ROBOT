/**
 * Mock authentication service for development and testing
 */

import { LoginCredentials } from '@/types/robot'

// Mock users database (not used in current auth flow, but kept for reference)
// const mockUsers = {
//     'admin': {
//         password: 'admin123',
//         user: {
//             id: 'admin',
//             scopes: ['robot:control', 'robot:read', 'admin:manage']
//         }
//     },
//     'operator': {
//         password: 'operator123',
//         user: {
//             id: 'operator',
//             scopes: ['robot:control', 'robot:read']
//         }
//     },
//     'viewer': {
//         password: 'viewer123',
//         user: {
//             id: 'viewer',
//             scopes: ['robot:read']
//         }
//     }
// }

// Mock API keys
const mockApiKeys = {
    'test-api-key-123': {
        user: {
            id: 'api-user',
            scopes: ['robot:control', 'robot:read']
        }
    },
    'admin-api-key-456': {
        user: {
            id: 'api-admin',
            scopes: ['robot:control', 'robot:read', 'admin:manage']
        }
    }
}

// Generate a simple JWT-like token (not secure, just for testing)
function generateMockToken(user: any): string {
    const header = btoa(JSON.stringify({ alg: 'HS256', typ: 'JWT' }))
    const payload = btoa(JSON.stringify({
        sub: user.id,
        scope: user.scopes,
        iat: Math.floor(Date.now() / 1000),
        exp: Math.floor(Date.now() / 1000) + (24 * 60 * 60) // 24 hours
    }))
    const signature = btoa('mock-signature')

    return `${header}.${payload}.${signature}`
}

export class MockAuthAPI {
    async login(credentials: LoginCredentials): Promise<{ token?: string; user?: any }> {
        // Simulate network delay
        await new Promise(resolve => setTimeout(resolve, 500))

        const { apiKey, userId, scopes = ['robot:read', 'robot:control'] } = credentials

        if (apiKey && userId) {
            const mockApiKey = mockApiKeys[apiKey as keyof typeof mockApiKeys]
            if (mockApiKey) {
                const user = {
                    id: userId,
                    scopes: scopes
                }
                return {
                    token: generateMockToken(user),
                    user: user
                }
            }
            throw new Error('Invalid API key')
        }

        throw new Error('API key and user ID required')
    }

    async validateApiKey(apiKey: string): Promise<{ valid: boolean; user?: any }> {
        // Simulate network delay
        await new Promise(resolve => setTimeout(resolve, 300))

        const mockApiKey = mockApiKeys[apiKey as keyof typeof mockApiKeys]
        if (mockApiKey) {
            return {
                valid: true,
                user: mockApiKey.user
            }
        }

        return { valid: false }
    }

    async refreshToken(token: string): Promise<{ token: string; user: any }> {
        // Simulate network delay
        await new Promise(resolve => setTimeout(resolve, 200))

        // For mock, just return a new token with same payload
        try {
            const payload = JSON.parse(atob(token.split('.')[1]))
            const user = { id: payload.sub, scopes: payload.scope }

            return {
                token: generateMockToken(user),
                user
            }
        } catch (error) {
            throw new Error('Invalid token')
        }
    }

    async logout(): Promise<void> {
        // Simulate network delay
        await new Promise(resolve => setTimeout(resolve, 100))
        // Mock logout - nothing to do
    }
}

// Export mock credentials for development
export const mockCredentials = {
    examples: [
        {
            apiKey: 'test-api-key-123',
            userId: 'test-user',
            scopes: ['robot:control', 'robot:read']
        },
        {
            apiKey: 'admin-api-key-456', 
            userId: 'admin-user',
            scopes: ['robot:control', 'robot:read', 'admin:manage']
        }
    ],
    apiKeys: Object.keys(mockApiKeys).map(key => ({
        key,
        scopes: mockApiKeys[key as keyof typeof mockApiKeys].user.scopes
    }))
}