'use client'

import { forwardRef, ButtonHTMLAttributes } from 'react'
import { cn } from '@/lib/utils'

export interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: 'default' | 'destructive' | 'outline' | 'secondary' | 'ghost' | 'link' | 'success' | 'warning'
  size?: 'default' | 'sm' | 'lg' | 'icon' | 'touch'
  isPressed?: boolean
  isHolding?: boolean
}

const Button = forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant = 'default', size = 'default', isPressed = false, isHolding = false, ...props }, ref) => {
    return (
      <button
        className={cn(
          // Base styles
          'inline-flex items-center justify-center whitespace-nowrap rounded-md text-sm font-medium ring-offset-background transition-all duration-150 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2 disabled:pointer-events-none disabled:opacity-50',
          // Touch-friendly base
          'touch-target select-none',
          // Variants
          {
            'bg-primary text-primary-foreground hover:bg-primary/90 active:bg-primary/80': variant === 'default',
            'bg-destructive text-destructive-foreground hover:bg-destructive/90 active:bg-destructive/80': variant === 'destructive',
            'border border-input bg-background hover:bg-accent hover:text-accent-foreground active:bg-accent/80': variant === 'outline',
            'bg-secondary text-secondary-foreground hover:bg-secondary/80 active:bg-secondary/60': variant === 'secondary',
            'hover:bg-accent hover:text-accent-foreground active:bg-accent/80': variant === 'ghost',
            'text-primary underline-offset-4 hover:underline': variant === 'link',
            'bg-success text-success-foreground hover:bg-success/90 active:bg-success/80': variant === 'success',
            'bg-warning text-warning-foreground hover:bg-warning/90 active:bg-warning/80': variant === 'warning',
          },
          // Sizes
          {
            'h-10 px-4 py-2': size === 'default',
            'h-9 rounded-md px-3': size === 'sm',
            'h-11 rounded-md px-8': size === 'lg',
            'h-10 w-10': size === 'icon',
            'h-14 w-14 text-lg': size === 'touch', // Touch-friendly size
          },
          // Press states
          {
            'transform scale-95 shadow-inner': isPressed,
            'animate-pulse-primary shadow-lg': isHolding,
          },
          className
        )}
        ref={ref}
        {...props}
      />
    )
  }
)
Button.displayName = 'Button'

export { Button }