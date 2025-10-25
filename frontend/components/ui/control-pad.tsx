'use client'

import { DirectionalButton } from './directional-button'
import { Button } from './button'
import { Direction } from '@/types/robot'

export interface ControlPadProps {
  levels: {
    up: number
    down: number
    left: number
    right: number
  }
  isHolding: {
    [key: string]: boolean
  }
  onDirectionPress: (direction: Direction) => void
  onDirectionRelease: (direction: Direction) => void
  onDirectionHold: (direction: Direction) => void
  onStop: () => void
  disabled?: boolean
  maxLevel?: number
  holdFrequency?: number
  holdInitialDelay?: number
}

export function ControlPad({
  levels,
  isHolding,
  onDirectionPress,
  onDirectionRelease,
  onDirectionHold,
  onStop,
  disabled = false,
  maxLevel = 10,
  holdFrequency = 15,
  holdInitialDelay = 200,
}: ControlPadProps) {
  const hasAnyLevel = Object.values(levels).some(level => level > 0)

  return (
    <div className="flex flex-col items-center space-y-4">
      {/* Top row - Forward button */}
      <div className="flex justify-center">
        <DirectionalButton
          direction="up"
          level={levels.up}
          isHolding={isHolding.up || false}
          onPress={onDirectionPress}
          onRelease={onDirectionRelease}
          onHold={onDirectionHold}
          maxLevel={maxLevel}
          disabled={disabled}
          holdFrequency={holdFrequency}
          holdInitialDelay={holdInitialDelay}
        />
      </div>

      {/* Middle row - Left, Stop, Right */}
      <div className="flex items-center space-x-4">
        <DirectionalButton
          direction="left"
          level={levels.left}
          isHolding={isHolding.left || false}
          onPress={onDirectionPress}
          onRelease={onDirectionRelease}
          onHold={onDirectionHold}
          maxLevel={maxLevel}
          disabled={disabled}
          holdFrequency={holdFrequency}
          holdInitialDelay={holdInitialDelay}
        />
        
        <Button
          variant={hasAnyLevel ? 'destructive' : 'secondary'}
          size="touch"
          onClick={onStop}
          disabled={disabled}
          className="h-16 w-16 rounded-full font-bold text-lg border-2"
        >
          STOP
        </Button>
        
        <DirectionalButton
          direction="right"
          level={levels.right}
          isHolding={isHolding.right || false}
          onPress={onDirectionPress}
          onRelease={onDirectionRelease}
          onHold={onDirectionHold}
          maxLevel={maxLevel}
          disabled={disabled}
          holdFrequency={holdFrequency}
          holdInitialDelay={holdInitialDelay}
        />
      </div>

      {/* Bottom row - Backward button */}
      <div className="flex justify-center">
        <DirectionalButton
          direction="down"
          level={levels.down}
          isHolding={isHolding.down || false}
          onPress={onDirectionPress}
          onRelease={onDirectionRelease}
          onHold={onDirectionHold}
          maxLevel={maxLevel}
          disabled={disabled}
          holdFrequency={holdFrequency}
          holdInitialDelay={holdInitialDelay}
        />
      </div>
    </div>
  )
}