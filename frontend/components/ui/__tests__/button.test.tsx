import { render, screen, fireEvent } from '@testing-library/react'
import { Button } from '../button'

describe('Button Component', () => {
    it('renders with default props', () => {
        render(<Button>Test Button</Button>)
        const button = screen.getByRole('button', { name: 'Test Button' })
        expect(button).toBeInTheDocument()
    })

    it('applies correct variant classes', () => {
        render(<Button variant="destructive">Delete</Button>)
        const button = screen.getByRole('button', { name: 'Delete' })
        expect(button).toHaveClass('bg-destructive')
    })

    it('applies pressed state correctly', () => {
        render(<Button isPressed>Pressed</Button>)
        const button = screen.getByRole('button', { name: 'Pressed' })
        expect(button).toHaveClass('transform', 'scale-95')
    })

    it('applies holding state correctly', () => {
        render(<Button isHolding>Holding</Button>)
        const button = screen.getByRole('button', { name: 'Holding' })
        expect(button).toHaveClass('animate-pulse-primary')
    })

    it('handles click events', () => {
        const handleClick = jest.fn()
        render(<Button onClick={handleClick}>Click Me</Button>)

        const button = screen.getByRole('button', { name: 'Click Me' })
        fireEvent.click(button)

        expect(handleClick).toHaveBeenCalledTimes(1)
    })

    it('is disabled when disabled prop is true', () => {
        render(<Button disabled>Disabled</Button>)
        const button = screen.getByRole('button', { name: 'Disabled' })
        expect(button).toBeDisabled()
    })

    it('applies touch size correctly', () => {
        render(<Button size="touch">Touch Button</Button>)
        const button = screen.getByRole('button', { name: 'Touch Button' })
        expect(button).toHaveClass('h-14', 'w-14')
    })

    it('applies success variant correctly', () => {
        render(<Button variant="success">Success</Button>)
        const button = screen.getByRole('button', { name: 'Success' })
        expect(button).toHaveClass('bg-success')
    })
})