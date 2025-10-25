'use client'

import { cn } from '@/lib/utils'

export interface VelocityDisplayProps {
  vx: number
  vy: number
  className?: string
  showLabels?: boolean
  precision?: number
}

export function VelocityDisplay({ 
  vx, 
  vy, 
  className,
  showLabels = true,
  precision = 2 
}: VelocityDisplayProps) {
  const formatVelocity = (value: number) => {
    return value.toFixed(precision)
  }

  const getVelocityColor = (value: number) => {
    const abs = Math.abs(value)
    if (abs === 0) return 'text-muted-foreground'
    if (abs < 0.3) return 'text-success'
    if (abs < 0.7) return 'text-warning'
    return 'text-destructive'
  }

  return (
    <div className={cn('space-y-4', className)}>
      {/* Velocity X */}
      <div className="space-y-2">
        {showLabels && (
          <label className="text-sm font-medium text-muted-foreground">
            Velocity X (Forward/Backward)
          </label>
        )}
        <div className="flex items-center space-x-2">
          <div className={cn(
            'text-3xl font-mono font-bold transition-colors duration-200',
            getVelocityColor(vx)
          )}>
            {vx >= 0 ? '+' : ''}{formatVelocity(vx)}
          </div>
          <span className="text-sm text-muted-foreground">m/s</span>
        </div>
        {/* Visual indicator bar */}
        <div className="relative h-2 bg-muted rounded-full overflow-hidden">
          <div 
            className={cn(
              'absolute top-0 h-full transition-all duration-200 rounded-full',
              vx > 0 ? 'bg-success' : vx < 0 ? 'bg-warning' : 'bg-muted-foreground'
            )}
            style={{
              width: `${Math.min(Math.abs(vx) * 100, 100)}%`,
              left: vx < 0 ? `${100 - Math.min(Math.abs(vx) * 100, 100)}%` : '0%'
            }}
          />
        </div>
      </div>

      {/* Velocity Y */}
      <div className="space-y-2">
        {showLabels && (
          <label className="text-sm font-medium text-muted-foreground">
            Velocity Y (Left/Right)
          </label>
        )}
        <div className="flex items-center space-x-2">
          <div className={cn(
            'text-3xl font-mono font-bold transition-colors duration-200',
            getVelocityColor(vy)
          )}>
            {vy >= 0 ? '+' : ''}{formatVelocity(vy)}
          </div>
          <span className="text-sm text-muted-foreground">m/s</span>
        </div>
        {/* Visual indicator bar */}
        <div className="relative h-2 bg-muted rounded-full overflow-hidden">
          <div 
            className={cn(
              'absolute top-0 h-full transition-all duration-200 rounded-full',
              vy > 0 ? 'bg-primary' : vy < 0 ? 'bg-secondary' : 'bg-muted-foreground'
            )}
            style={{
              width: `${Math.min(Math.abs(vy) * 100, 100)}%`,
              left: vy < 0 ? `${100 - Math.min(Math.abs(vy) * 100, 100)}%` : '0%'
            }}
          />
        </div>
      </div>
    </div>
  )
}

export interface SpeedLevelDisplayProps {
  levels: {
    up: number
    down: number
    left: number
    right: number
  }
  maxLevel?: number
  className?: string
}

export function SpeedLevelDisplay({ 
  levels, 
  maxLevel = 10, 
  className 
}: SpeedLevelDisplayProps) {
  const levelItems = [
    { key: 'up', label: 'Forward', value: levels.up, color: 'bg-success' },
    { key: 'down', label: 'Backward', value: levels.down, color: 'bg-warning' },
    { key: 'left', label: 'Left', value: levels.left, color: 'bg-secondary' },
    { key: 'right', label: 'Right', value: levels.right, color: 'bg-primary' },
  ]

  return (
    <div className={cn('space-y-3', className)}>
      <h3 className="text-sm font-medium text-muted-foreground">Speed Levels</h3>
      <div className="grid grid-cols-2 gap-3">
        {levelItems.map(({ key, label, value, color }) => (
          <div key={key} className="space-y-2">
            <div className="flex justify-between items-center">
              <span className="text-xs font-medium">{label}</span>
              <span className="text-xs font-mono">{value}/{maxLevel}</span>
            </div>
            <div className="h-2 bg-muted rounded-full overflow-hidden">
              <div 
                className={cn(
                  'h-full transition-all duration-300 rounded-full',
                  value > 0 ? color : 'bg-muted-foreground/20'
                )}
                style={{ width: `${(value / maxLevel) * 100}%` }}
              />
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}