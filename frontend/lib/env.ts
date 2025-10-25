/**
 * Environment configuration for the frontend application
 */

// Debug environment variables
console.log('Environment variables at build time:', {
  NEXT_PUBLIC_VX_MAX: process.env.NEXT_PUBLIC_VX_MAX,
  NEXT_PUBLIC_VY_MAX: process.env.NEXT_PUBLIC_VY_MAX,
});

export const env = {
  // API URLs
  API_URL: process.env.NEXT_PUBLIC_API_URL || 'http://localhost:3001',
  WS_URL: process.env.NEXT_PUBLIC_WS_URL || 'ws://localhost:3001',

  // Authentication
  AUTH_ENABLED: process.env.NEXT_PUBLIC_AUTH_ENABLED === 'true',
  JWT_STORAGE_KEY: 'teleop_jwt_token',
  API_KEY_STORAGE_KEY: 'teleop_api_key',

  // WebSocket
  WS_ENABLED: process.env.NEXT_PUBLIC_WS_ENABLED !== 'false', // Default enabled

  // Velocity limits
  VX_MAX: parseFloat(process.env.NEXT_PUBLIC_VX_MAX || '1.0'),
  VY_MAX: parseFloat(process.env.NEXT_PUBLIC_VY_MAX || '1.0'),

  // Environment
  NODE_ENV: process.env.NODE_ENV || 'development',

  // Feature flags
  isDevelopment: process.env.NODE_ENV === 'development',
  isProduction: process.env.NODE_ENV === 'production',
} as const

// Validate required environment variables (currently unused but kept for future use)
// const requiredEnvVars = ['NEXT_PUBLIC_API_URL', 'NEXT_PUBLIC_WS_URL'] as const

export function validateEnv() {
  // Only validate in browser runtime, not during build
  if (typeof window === 'undefined') return

  // Since the env object already has the values with fallbacks, just check if they're using defaults
  const hasApiUrl = env.API_URL !== 'http://localhost:3001' || process.env.NEXT_PUBLIC_API_URL
  const hasWsUrl = env.WS_URL !== 'ws://localhost:3001' || process.env.NEXT_PUBLIC_WS_URL

  console.log('Environment validation:', {
    API_URL: env.API_URL,
    WS_URL: env.WS_URL,
    AUTH_ENABLED: env.AUTH_ENABLED,
    hasApiUrl,
    hasWsUrl
  })

  // Only warn in production if using default values without explicit env vars
  if (env.isProduction && (!hasApiUrl || !hasWsUrl)) {
    console.warn('Using default API URLs in production. Consider setting NEXT_PUBLIC_API_URL and NEXT_PUBLIC_WS_URL')
  }
}

// Auto-validate in browser runtime (disabled for now since env vars are working)
// if (typeof window !== 'undefined') {
//   validateEnv()
// }