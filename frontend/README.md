# Web Teleop Robot Frontend

A Next.js application for controlling robots remotely through a web interface.

## Features

- **Modern UI**: Built with Next.js 14, TypeScript, and Tailwind CSS
- **Responsive Design**: Mobile-first approach with touch-friendly controls
- **Theme Support**: Dark/light mode with system preference detection
- **Type Safety**: Full TypeScript support with strict type checking
- **Performance**: Optimized build with static generation and code splitting

## Getting Started

### Prerequisites

- Node.js 18+ 
- npm or yarn

### Installation

```bash
# Install dependencies
npm install

# Copy environment variables
cp .env.local.example .env.local
```

### Development

```bash
# Start development server
npm run dev

# Build for production
npm run build

# Start production server
npm start

# Run linting
npm run lint

# Type checking
npm run type-check
```

## Project Structure

```
frontend/
├── app/                    # Next.js App Router
│   ├── globals.css        # Global styles
│   ├── layout.tsx         # Root layout
│   └── page.tsx           # Home page
├── components/            # React components
│   ├── robot-control-panel.tsx
│   ├── theme-provider.tsx
│   └── theme-toggle.tsx
├── lib/                   # Utility functions
│   ├── env.ts            # Environment configuration
│   ├── responsive.ts     # Responsive utilities
│   └── utils.ts          # General utilities
├── types/                 # TypeScript definitions
│   ├── index.ts
│   └── robot.ts
├── next.config.js         # Next.js configuration
├── tailwind.config.ts     # Tailwind CSS configuration
└── tsconfig.json         # TypeScript configuration
```

## Environment Variables

Create a `.env.local` file with the following variables:

```bash
# API Configuration
NEXT_PUBLIC_API_URL=http://localhost:3001
NEXT_PUBLIC_WS_URL=ws://localhost:3001

# Development Configuration
NODE_ENV=development
```

## Styling

The application uses Tailwind CSS with a custom design system:

- **Colors**: Primary blue, success green, warning amber, danger red
- **Typography**: Inter font family
- **Spacing**: 8px grid system
- **Animations**: Smooth transitions with easing functions
- **Responsive**: Mobile-first breakpoints

## Theme System

The application supports:

- Light mode
- Dark mode  
- System preference detection
- Persistent theme selection
- High contrast mode support

## Accessibility

- Keyboard navigation support
- Screen reader compatibility
- High contrast mode
- Touch-friendly targets (44px minimum)
- ARIA labels and roles

## Performance

- Static generation where possible
- Code splitting and lazy loading
- Optimized images and fonts
- Minimal bundle size
- Fast refresh in development

## Browser Support

- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Contributing

1. Follow the existing code style
2. Add TypeScript types for new features
3. Test responsive design on multiple devices
4. Ensure accessibility compliance
5. Update documentation as needed