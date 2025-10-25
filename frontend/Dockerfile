FROM node:20-alpine AS base
RUN apk add --no-cache libc6-compat
WORKDIR /app

# Dependencies stage
FROM base AS deps
# Copy root package.json and workspace files
COPY package*.json ./
COPY shared/package*.json ./shared/
COPY frontend/package*.json ./frontend/

# Copy shared source code
COPY shared/ ./shared/

# Install root dependencies first (for workspace)
RUN npm install

# Build shared package
RUN cd shared && npm run build

# Copy frontend source
COPY frontend/ ./frontend/

# Install frontend dependencies
RUN cd frontend && npm ci

# Build stage
FROM base AS builder
WORKDIR /app

# Copy everything from deps stage
COPY --from=deps /app ./

# Set environment variables
ENV NEXT_TELEMETRY_DISABLED=1
ENV NODE_OPTIONS="--max-old-space-size=4096"

# Build the frontend
RUN cd frontend && npm run build

# Production stage
FROM base AS runner
WORKDIR /app

ENV NODE_ENV=production
ENV NEXT_TELEMETRY_DISABLED=1

RUN addgroup --system --gid 1001 nodejs
RUN adduser --system --uid 1001 nextjs

# Copy built frontend application
COPY --from=builder --chown=nextjs:nodejs /app/frontend ./

USER nextjs

EXPOSE 5000

ENV PORT=5000
ENV HOSTNAME="0.0.0.0"

CMD ["npm", "start"]