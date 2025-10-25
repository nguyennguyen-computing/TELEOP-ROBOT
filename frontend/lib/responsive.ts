/**
 * Responsive design utilities and breakpoints
 */

export const breakpoints = {
  xs: '475px',
  sm: '640px',
  md: '768px',
  lg: '1024px',
  xl: '1280px',
  '2xl': '1536px',
} as const

export type Breakpoint = keyof typeof breakpoints

/**
 * Check if the current viewport matches a breakpoint
 */
export function useMediaQuery(query: string): boolean {
  if (typeof window === 'undefined') return false

  const mediaQuery = window.matchMedia(query)
  return mediaQuery.matches
}

/**
 * Get responsive classes based on screen size
 */
export function getResponsiveClasses(
  base: string,
  responsive: Partial<Record<Breakpoint, string>> = {}
): string {
  const classes = [base]

  Object.entries(responsive).forEach(([breakpoint, className]) => {
    if (className) {
      classes.push(`${breakpoint}:${className}`)
    }
  })

  return classes.join(' ')
}

/**
 * Common responsive patterns
 */
export const responsivePatterns = {
  // Grid layouts
  grid: {
    single: 'grid-cols-1',
    double: 'grid-cols-1 md:grid-cols-2',
    triple: 'grid-cols-1 md:grid-cols-2 lg:grid-cols-3',
    quad: 'grid-cols-1 sm:grid-cols-2 lg:grid-cols-4',
  },

  // Spacing
  spacing: {
    container: 'px-4 sm:px-6 lg:px-8',
    section: 'py-8 sm:py-12 lg:py-16',
    gap: 'gap-4 sm:gap-6 lg:gap-8',
  },

  // Typography
  text: {
    heading: 'text-2xl sm:text-3xl lg:text-4xl',
    subheading: 'text-lg sm:text-xl lg:text-2xl',
    body: 'text-sm sm:text-base',
  },

  // Touch targets
  touch: {
    button: 'min-h-[44px] min-w-[44px] touch-target',
    input: 'min-h-[44px] touch-target',
  },
} as const