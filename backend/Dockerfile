# ===========================
# 1️⃣ Build stage
# ===========================
FROM node:20-alpine AS builder
WORKDIR /app

# First, build shared package
COPY shared ../shared
WORKDIR /shared
RUN npm install --legacy-peer-deps
RUN npm run build

# Now build backend
WORKDIR /app
COPY backend/package*.json ./
COPY backend/tsconfig.json ./

# Install all dependencies (including dev dependencies for build)
RUN npm install --legacy-peer-deps

# Copy source code
COPY backend/src ./src

# Build TypeScript to JavaScript
RUN npm run build

# ===========================
# 2️⃣ Runtime image
# ===========================
FROM node:20-alpine AS runner
WORKDIR /app

# Copy package files and install production dependencies only
COPY backend/package*.json ./
RUN npm install --omit=dev --legacy-peer-deps

# Copy built code from builder stage
COPY --from=builder /app/dist ./dist

# Copy built shared package from builder stage
COPY --from=builder /shared ../shared

# Set environment variables
ENV NODE_ENV=production
ENV NODE_PORT=3001

# Create non-root user for safety
RUN addgroup -S nodeapp && adduser -S nodeuser -G nodeapp
RUN chown -R nodeuser:nodeapp /app
USER nodeuser

# Expose API port
EXPOSE 3001

# Start server
CMD ["node", "dist/index.js"]