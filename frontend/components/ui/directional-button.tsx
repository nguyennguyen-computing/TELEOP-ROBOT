'use client'

import { forwardRef, ButtonHTMLAttributes, useCallback, useRef, useEffect } from 'react'
import { Button } from './button'
import { cn } from '@/lib/utils'
import { Direction } from '@/types/robot'

export interface DirectionalButtonProps extends Omit<ButtonHTMLAttributes<HTMLButtonElement>, 'onMouseDown' | 'onMouseUp' | 'onTouchStart' | 'onTouchEnd'> {
  direction: Direction
  level: number
  isHolding: boolean
  onPress: (direction: Direction) => void
  onRelease: (direction: Direction) => void
  onHold: (direction: Direction) => void
  maxLevel?: number
  holdFrequency?: number
  holdInitialDelay?: number
}

const directionIcons = {
  up: '↑',
  down: '↓',
  left: '←',
  right: '→',
}

const directionLabels = {
  up: 'Forward',
  down: 'Backward',
  left: 'Left',
  right: 'Right',
}

const DirectionalButton = forwardRef<HTMLButtonElement, DirectionalButtonProps>(
  ({
    direction,
    level,
    isHolding,
    onPress,
    onRelease,
    onHold,
    maxLevel = 10,
    holdFrequency = 15,
    holdInitialDelay = 200,
    className,
    disabled,
    ...props
  }, ref) => {
    const holdIntervalRef = useRef<NodeJS.Timeout | null>(null)
    const holdTimeoutRef = useRef<NodeJS.Timeout | null>(null)
    const isMouseDownRef = useRef(false)
    const isTouchActiveRef = useRef(false)

    const startHold = useCallback(() => {
      if (disabled) return

      onPress(direction)

      // Start hold after initial delay
      holdTimeoutRef.current = setTimeout(() => {
        // Start continuous hold at configured frequency (10-20Hz requirement)
        const holdInterval = Math.round(1000 / holdFrequency)
        holdIntervalRef.current = setInterval(() => {
          onHold(direction)
        }, holdInterval)
      }, holdInitialDelay)
    }, [direction, onPress, onHold, disabled, holdFrequency, holdInitialDelay])

    const stopHold = useCallback(() => {
      if (holdTimeoutRef.current) {
        clearTimeout(holdTimeoutRef.current)
        holdTimeoutRef.current = null
      }
      if (holdIntervalRef.current) {
        clearInterval(holdIntervalRef.current)
        holdIntervalRef.current = null
      }

      if (!disabled) {
        onRelease(direction)
      }

      isMouseDownRef.current = false
      isTouchActiveRef.current = false
    }, [direction, onRelease, disabled])

    // Mouse events
    const handleMouseDown = useCallback((e: React.MouseEvent) => {
      e.preventDefault()
      if (disabled || isTouchActiveRef.current) return

      isMouseDownRef.current = true
      startHold()
    }, [startHold, disabled])

    const handleMouseUp = useCallback((e: React.MouseEvent) => {
      e.preventDefault()
      if (!isMouseDownRef.current) return
      stopHold()
    }, [stopHold])

    const handleMouseLeave = useCallback((e: React.MouseEvent) => {
      e.preventDefault()
      if (!isMouseDownRef.current) return
      stopHold()
    }, [stopHold])

    // Touch events
    const handleTouchStart = useCallback((e: React.TouchEvent) => {
      e.preventDefault()
      if (disabled) return

      isTouchActiveRef.current = true
      startHold()
    }, [startHold, disabled])

    const handleTouchEnd = useCallback((e: React.TouchEvent) => {
      e.preventDefault()
      if (!isTouchActiveRef.current) return
      stopHold()
    }, [stopHold])

    // Cleanup on unmount
    useEffect(() => {
      return () => {
        if (holdTimeoutRef.current) clearTimeout(holdTimeoutRef.current)
        if (holdIntervalRef.current) clearInterval(holdIntervalRef.current)
      }
    }, [])

    // Global mouse up handler to catch mouse up outside button
    useEffect(() => {
      const handleGlobalMouseUp = () => {
        if (isMouseDownRef.current) {
          stopHold()
        }
      }

      const handleGlobalTouchEnd = () => {
        if (isTouchActiveRef.current) {
          stopHold()
        }
      }

      document.addEventListener('mouseup', handleGlobalMouseUp)
      document.addEventListener('touchend', handleGlobalTouchEnd)
      document.addEventListener('touchcancel', handleGlobalTouchEnd)

      return () => {
        document.removeEventListener('mouseup', handleGlobalMouseUp)
        document.removeEventListener('touchend', handleGlobalTouchEnd)
        document.removeEventListener('touchcancel', handleGlobalTouchEnd)
      }
    }, [stopHold])

    const levelPercentage = (level / maxLevel) * 100

    return (
      <div className="relative">
        <Button
          ref={ref}
          variant={level > 0 ? 'default' : 'outline'}
          size="touch"
          className={cn(
            'relative overflow-hidden transition-all duration-150',
            'flex flex-col items-center justify-center gap-1',
            'border-2 font-semibold',
            {
              'border-primary bg-primary text-primary-foreground': level > 0,
              'border-border hover:border-primary/50': level === 0,
              'shadow-lg ring-2 ring-primary/20': isHolding,
            },
            className
          )}
          isPressed={isHolding}
          isHolding={isHolding}
          disabled={disabled}
          onMouseDown={handleMouseDown}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseLeave}
          onTouchStart={handleTouchStart}
          onTouchEnd={handleTouchEnd}
          {...props}
        >
          {/* Level indicator background */}
          {level > 0 && (
            <div
              className="absolute inset-0 bg-primary/20 transition-all duration-200"
              style={{
                height: `${levelPercentage}%`,
                bottom: 0,
              }}
            />
          )}

          {/* Content */}
          <div className="relative z-10 flex flex-col items-center justify-center">
            <span className="text-2xl leading-none">
              {directionIcons[direction]}
            </span>
            <span className="text-xs font-medium">
              {directionLabels[direction]}
            </span>
          </div>

          {/* Level indicator */}
          {level > 0 && (
            <div className="absolute bottom-1 right-1 z-10">
              <span className="text-xs font-bold bg-background/80 text-foreground px-1 rounded">
                {level}
              </span>
            </div>
          )}
        </Button>
      </div>
    )
  }
)

DirectionalButton.displayName = 'DirectionalButton'

export { DirectionalButton }