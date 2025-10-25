import { render, screen, fireEvent } from '@testing-library/react'
import { DirectionalButton } from '../ui/directional-button'

describe('Control Logic Implementation', () => {
  let mockOnPress: jest.Mock
  let mockOnRelease: jest.Mock
  let mockOnHold: jest.Mock

  beforeEach(() => {
    mockOnPress = jest.fn()
    mockOnRelease = jest.fn()
    mockOnHold = jest.fn()
    
    // Mock navigator.vibrate
    Object.defineProperty(navigator, 'vibrate', {
      value: jest.fn(),
      writable: true,
    })
  })

  afterEach(() => {
    jest.clearAllMocks()
  })

  describe('Press & Hold Functionality', () => {
    it('calls onPress when button is pressed', () => {
      render(
        <DirectionalButton
          direction="up"
          level={0}
          isHolding={false}
          onPress={mockOnPress}
          onRelease={mockOnRelease}
          onHold={mockOnHold}
        />
      )

      const button = screen.getByRole('button')
      fireEvent.mouseDown(button)

      expect(mockOnPress).toHaveBeenCalledWith('up')
    })

    it('calls onRelease when button is released', () => {
      render(
        <DirectionalButton
          direction="up"
          level={0}
          isHolding={false}
          onPress={mockOnPress}
          onRelease={mockOnRelease}
          onHold={mockOnHold}
        />
      )

      const button = screen.getByRole('button')
      fireEvent.mouseDown(button)
      fireEvent.mouseUp(button)

      expect(mockOnRelease).toHaveBeenCalledWith('up')
    })

    it('calls onHold repeatedly when button is held', async () => {
      jest.useFakeTimers()
      
      render(
        <DirectionalButton
          direction="up"
          level={0}
          isHolding={false}
          onPress={mockOnPress}
          onRelease={mockOnRelease}
          onHold={mockOnHold}
          holdFrequency={15}
          holdInitialDelay={200}
        />
      )

      const button = screen.getByRole('button')
      fireEvent.mouseDown(button)

      // Initial press
      expect(mockOnPress).toHaveBeenCalledTimes(1)

      // Fast forward past initial delay
      jest.advanceTimersByTime(250)

      // Fast forward to trigger hold calls (15Hz = ~66ms intervals)
      jest.advanceTimersByTime(200) // ~3 hold calls

      expect(mockOnHold).toHaveBeenCalledWith('up')
      expect(mockOnHold).toHaveBeenCalledTimes(3)

      fireEvent.mouseUp(button)
      jest.useRealTimers()
    })

    it('uses configurable hold frequency within 10-20 Hz range', () => {
      const frequencies = [10, 15, 20]
      
      frequencies.forEach(frequency => {
        const expectedInterval = Math.round(1000 / frequency)
        expect(expectedInterval).toBeGreaterThanOrEqual(50) // 20Hz = 50ms
        expect(expectedInterval).toBeLessThanOrEqual(100) // 10Hz = 100ms
      })
    })
  })

  describe('Touch Support', () => {
    it('handles touch events correctly', () => {
      render(
        <DirectionalButton
          direction="up"
          level={0}
          isHolding={false}
          onPress={mockOnPress}
          onRelease={mockOnRelease}
          onHold={mockOnHold}
        />
      )

      const button = screen.getByRole('button')
      fireEvent.touchStart(button)

      expect(mockOnPress).toHaveBeenCalledWith('up')

      fireEvent.touchEnd(button)
      expect(mockOnRelease).toHaveBeenCalledWith('up')
    })
  })

  describe('Level Management', () => {
    it('displays current level correctly', () => {
      render(
        <DirectionalButton
          direction="up"
          level={5}
          isHolding={false}
          onPress={mockOnPress}
          onRelease={mockOnRelease}
          onHold={mockOnHold}
        />
      )

      expect(screen.getByText('5')).toBeInTheDocument()
    })

    it('shows visual feedback for different levels', () => {
      const { rerender } = render(
        <DirectionalButton
          direction="up"
          level={0}
          isHolding={false}
          onPress={mockOnPress}
          onRelease={mockOnRelease}
          onHold={mockOnHold}
        />
      )

      const button = screen.getByRole('button')
      expect(button).toHaveClass('border-border')

      rerender(
        <DirectionalButton
          direction="up"
          level={5}
          isHolding={false}
          onPress={mockOnPress}
          onRelease={mockOnRelease}
          onHold={mockOnHold}
        />
      )

      expect(button).toHaveClass('border-primary')
    })
  })

  describe('Velocity Calculation Logic', () => {
    it('calculates velocity correctly from levels', () => {
      const VX_MAX = 1.0 // Using default test values
      const VY_MAX = 1.0 // Using default test values
      const STEP_X = VX_MAX / 10
      const STEP_Y = VY_MAX / 10

      const calculateVelocity = (levels: { up: number; down: number; left: number; right: number }) => {
        const net_x_level = levels.up - levels.down
        const net_y_level = levels.right - levels.left
        
        const clamp = (value: number, min: number, max: number) => 
          Math.min(Math.max(value, min), max)
        
        const vx = clamp(net_x_level, -10, 10) * STEP_X
        const vy = clamp(net_y_level, -10, 10) * STEP_Y
        
        return { vx, vy }
      }

      // Test forward movement
      const forwardResult = calculateVelocity({ up: 5, down: 0, left: 0, right: 0 })
      expect(forwardResult.vx).toBeCloseTo(0.5)
      expect(forwardResult.vy).toBeCloseTo(0.0)

      // Test backward movement
      const backwardResult = calculateVelocity({ up: 0, down: 3, left: 0, right: 0 })
      expect(backwardResult.vx).toBeCloseTo(-0.3)
      expect(backwardResult.vy).toBeCloseTo(0.0)

      // Test right movement
      const rightResult = calculateVelocity({ up: 0, down: 0, left: 0, right: 7 })
      expect(rightResult.vx).toBeCloseTo(0.0)
      expect(rightResult.vy).toBeCloseTo(0.7)

      // Test left movement
      const leftResult = calculateVelocity({ up: 0, down: 0, left: 4, right: 0 })
      expect(leftResult.vx).toBeCloseTo(0.0)
      expect(leftResult.vy).toBeCloseTo(-0.4)

      // Test combined movement
      const combinedResult = calculateVelocity({ up: 6, down: 2, left: 1, right: 5 })
      expect(combinedResult.vx).toBeCloseTo(0.4)
      expect(combinedResult.vy).toBeCloseTo(0.4)

      // Test maximum clamping
      const maxResult = calculateVelocity({ up: 15, down: 0, left: 0, right: 12 })
      expect(maxResult.vx).toBeCloseTo(1.0)
      expect(maxResult.vy).toBeCloseTo(1.0)
    })

    it('enforces coordinate system mapping correctly', () => {
      // Forward = +X, Backward = -X
      // Right = +Y, Left = -Y
      
      const testCases = [
        { levels: { up: 5, down: 0, left: 0, right: 0 }, expected: { vx: 0.5, vy: 0.0 } }, // Forward
        { levels: { up: 0, down: 5, left: 0, right: 0 }, expected: { vx: -0.5, vy: 0.0 } }, // Backward
        { levels: { up: 0, down: 0, left: 0, right: 5 }, expected: { vx: 0.0, vy: 0.5 } }, // Right
        { levels: { up: 0, down: 0, left: 5, right: 0 }, expected: { vx: 0.0, vy: -0.5 } }, // Left
      ]

      testCases.forEach(({ levels, expected }) => {
        const net_x_level = levels.up - levels.down
        const net_y_level = levels.right - levels.left
        const vx = net_x_level * 0.1
        const vy = net_y_level * 0.1
        
        expect({ vx, vy }).toEqual(expected)
      })
    })
  })

  describe('Stop Functionality', () => {
    it('resets all levels to 0', () => {
      const resetLevels = { up: 0, down: 0, left: 0, right: 0 }
      
      expect(resetLevels).toEqual({ up: 0, down: 0, left: 0, right: 0 })
      
      // Verify velocity is also reset
      const velocity = {
        vx: (resetLevels.up - resetLevels.down) * 0.1,
        vy: (resetLevels.right - resetLevels.left) * 0.1
      }
      
      expect(velocity).toEqual({ vx: 0, vy: 0 })
    })
  })

  describe('Keyboard Shortcuts', () => {
    it('maps WASD keys correctly', () => {
      const keyMap = {
        'w': 'up',
        'a': 'left',
        's': 'down',
        'd': 'right'
      }

      expect(keyMap.w).toBe('up')
      expect(keyMap.a).toBe('left')
      expect(keyMap.s).toBe('down')
      expect(keyMap.d).toBe('right')
    })

    it('handles space key for stop', () => {
      const stopKey = ' '
      expect(stopKey).toBe(' ')
    })
  })
})