'use client'

import { cn } from '@/lib/utils'
import type { ConnectionStatus } from '@/types/robot'

export interface ConnectionStatusProps {
  status: ConnectionStatus
  className?: string
  showLabel?: boolean
  size?: 'sm' | 'md' | 'lg'
}

const statusConfig = {
  connected: {
    color: 'bg-green-500',
    animation: 'animate-pulse-success',
    label: 'Connected',
    description: 'Robot is online and ready',
  },
  connecting: {
    color: 'bg-yellow-500',
    animation: 'animate-pulse',
    label: 'Connecting',
    description: 'Establishing connection...',
  },
  disconnected: {
    color: 'bg-red-500',
    animation: '',
    label: 'Disconnected',
    description: 'Robot is offline',
  },
}

const sizeConfig = {
  sm: 'w-2 h-2',
  md: 'w-3 h-3',
  lg: 'w-4 h-4',
}

export function ConnectionStatus({ 
  status, 
  className,
  showLabel = true,
  size = 'md'
}: ConnectionStatusProps) {
  const config = statusConfig[status] || statusConfig.disconnected
  
  return (
    <div className={cn('flex items-center space-x-2', className)}>
      <div 
        className={cn(
          'rounded-full transition-colors duration-200',
          sizeConfig[size],
          config.color,
          config.animation
        )}
      />
      {showLabel && (
        <div className="flex flex-col">
          <span className="text-sm font-medium capitalize">
            {config.label}
          </span>
          <span className="text-xs text-muted-foreground">
            {config.description}
          </span>
        </div>
      )}
    </div>
  )
}

export interface ConnectionInfoProps {
  status: ConnectionStatus
  lastUpdate?: Date
  latency?: number
  className?: string
}

export function ConnectionInfo({ 
  status, 
  lastUpdate, 
  latency,
  className 
}: ConnectionInfoProps) {
  const formatLatency = (ms: number) => {
    if (ms < 1000) return `${ms}ms`
    return `${(ms / 1000).toFixed(1)}s`
  }

  const formatLastUpdate = (date: Date) => {
    const now = new Date()
    const diff = now.getTime() - date.getTime()
    
    if (diff < 1000) return 'Just now'
    if (diff < 60000) return `${Math.floor(diff / 1000)}s ago`
    if (diff < 3600000) return `${Math.floor(diff / 60000)}m ago`
    return date.toLocaleTimeString()
  }

  const getLatencyQuality = (ms: number) => {
    if (ms < 50) return { label: 'Excellent', color: 'text-green-600' }
    if (ms < 100) return { label: 'Good', color: 'text-green-500' }
    if (ms < 200) return { label: 'Fair', color: 'text-yellow-500' }
    if (ms < 500) return { label: 'Poor', color: 'text-orange-500' }
    return { label: 'Bad', color: 'text-red-500' }
  }

  return (
    <div className={cn('space-y-3', className)}>
      <ConnectionStatus status={status} showLabel={true} size="lg" />
      
      <div className="grid grid-cols-2 gap-4 text-sm">
        {latency !== undefined && latency > 0 && (
          <div>
            <span className="text-muted-foreground">Latency:</span>
            <div className="flex items-center space-x-2">
              <div className={cn('font-mono font-medium', getLatencyQuality(latency).color)}>
                {formatLatency(latency)}
              </div>
              <span className={cn('text-xs', getLatencyQuality(latency).color)}>
                ({getLatencyQuality(latency).label})
              </span>
            </div>
          </div>
        )}
        
        {lastUpdate && (
          <div>
            <span className="text-muted-foreground">Last Update:</span>
            <div className="font-mono font-medium">
              {formatLastUpdate(lastUpdate)}
            </div>
          </div>
        )}
      </div>

      {/* Real-time connection indicator */}
      {status === 'connected' && (
        <div className="flex items-center space-x-2 text-xs text-muted-foreground">
          <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse" />
          <span>Real-time WebSocket connection active</span>
        </div>
      )}
    </div>
  )
}