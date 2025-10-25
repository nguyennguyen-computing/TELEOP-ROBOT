# Local Development Setup Guide

## 🎯 Overview

This guide explains how to set up the teleop robot system for local development. The system uses a centralized `.env` configuration that automatically syncs to individual service `.env` files.

## 📁 Environment File Structure

```
project-root/
├── .env                    # ← Master configuration (39 variables)
├── .env.example           # ← Template for new setups
├── backend/.env           # ← Auto-generated for local backend dev
├── frontend/.env.local    # ← Auto-generated for local frontend dev
└── sync-frontend-env.py   # ← Sync script
```

## 🚀 Quick Setup

### 1. Automated Setup (Recommended)
```bash
# Run the setup script
chmod +x setup-dev.sh
./setup-dev.sh
```

This will:
- ✅ Validate root `.env` file exists
- ✅ Sync backend and frontend environment files
- ✅ Validate all environment variables
- ✅ Confirm system is ready

### 2. Manual Setup
```bash
# 1. Ensure root .env exists
cp .env.example .env
# Edit .env as needed

# 2. Sync environment files
python3 sync-frontend-env.py

# 3. Validate configuration
python3 validate-env.py
```

## 🔄 Environment Sync Process

The sync script (`sync-frontend-env.py`) automatically:

### Backend Sync (`backend/.env`)
- Copies 14 variables from root `.env`
- Overrides URLs for localhost development:
  - `FLEET_API_URL` → `http://localhost:8000`
  - `MONGO_URI` → `mongodb://localhost:27017/teleop_db`
  - `ZENOH_LOCATOR` → `tcp/localhost:7447`

### Frontend Sync (`frontend/.env.local`)
- Copies 5 variables from root `.env`
- Overrides URLs for localhost development:
  - `NEXT_PUBLIC_API_URL` → `http://localhost:3001`
  - `NEXT_PUBLIC_WS_URL` → `ws://localhost:3001`

## 🏃‍♂️ Running Services Locally

### Option 1: Full Docker Stack
```bash
# Start all services in Docker
docker-compose up -d

# Access services:
# - Frontend: http://localhost:3000
# - Backend API: http://localhost:3001
# - Fleet API: http://localhost:8000
# - Zenoh REST: http://localhost:8080
```

### Option 2: Mixed Local/Docker
```bash
# Start infrastructure (MongoDB, Zenoh, etc.)
docker-compose up -d mongo zenoh fleet-api

# Run backend locally
cd backend
npm install
npm run dev

# Run frontend locally (in another terminal)
cd frontend
npm install
npm run dev
```

### Option 3: Full Local Development
```bash
# Start MongoDB and Zenoh
docker-compose up -d mongo zenoh

# Run fleet server locally
cd fleet-server
pip install -r requirements.txt
python run.py

# Run backend locally
cd backend
npm install
npm run dev

# Run frontend locally
cd frontend
npm install
npm run dev
```

## 🔧 Development Workflow

### Making Configuration Changes
1. **Edit root `.env` file** (single source of truth)
2. **Run sync script**: `python3 sync-frontend-env.py`
3. **Restart services** that need the new config

### Adding New Environment Variables
1. **Add to root `.env`**
2. **Update sync script** if needed for backend/frontend
3. **Update validation script** (`validate-env.py`)
4. **Update documentation**

## 🔍 Troubleshooting

### Backend Can't Find Environment Variables
```bash
# Check if backend/.env exists
ls -la backend/.env

# If missing, run sync
python3 sync-frontend-env.py

# Verify variables are loaded
cd backend && npm run dev
```

### Frontend Environment Issues
```bash
# Check if frontend/.env.local exists
ls -la frontend/.env.local

# If missing, run sync
python3 sync-frontend-env.py

# Verify Next.js loads the file
cd frontend && npm run dev
# Look for "Environments: .env.local" in output
```

### Docker vs Local URL Conflicts
The sync script automatically handles URL differences:
- **Docker**: Uses service names (`http://fleet-api:8000`)
- **Local**: Uses localhost (`http://localhost:8000`)

### Environment Validation Failures
```bash
# Run validation to see what's missing
python3 validate-env.py

# Common issues:
# - Missing variables in root .env
# - Typos in variable names
# - Missing quotes around values with spaces
```

## 📊 Environment Variables by Service

### Backend (14 variables)
```bash
NODE_PORT=3001
FLEET_API_URL=http://localhost:8000
MONGO_URI=mongodb://localhost:27017/teleop_db
CORS_ORIGIN=http://localhost:3000
VX_MAX=1.0
VY_MAX=1.0
ZENOH_LOCATOR=tcp/localhost:7447
Z_KEY_CMD_VEL=cmd_vel
AUTH_ENABLED=true
API_KEYS=test-api-key-123,admin-key-456
JWT_SECRET=your-super-secret-jwt-key-here-change-in-production
JWT_EXPIRES_IN=24h
RATE_LIMIT_WINDOW_MS=900000
RATE_LIMIT_MAX_REQUESTS=100
```

### Frontend (5 variables)
```bash
NEXT_PUBLIC_API_URL=http://localhost:3001
NEXT_PUBLIC_WS_URL=ws://localhost:3001
NEXT_PUBLIC_AUTH_ENABLED=true
NEXT_PUBLIC_WS_ENABLED=false
NODE_ENV=development
```

## 🚨 Important Notes

### Git Ignore
The following files are automatically ignored by git:
- `backend/.env` (auto-generated)
- `frontend/.env.local` (auto-generated)

### Security
- **Never commit** local `.env` files
- **Change default secrets** in production
- **Use environment-specific values** for different deployments

### Sync Frequency
- **Run sync** after changing root `.env`
- **Run sync** when switching branches
- **Run sync** when setting up new development environment

## 🎉 Success Indicators

When everything is set up correctly, you should see:

### Backend Startup
```
✅ Environment variables loaded
✅ Connected to MongoDB
✅ Server running on port 3001
```

### Frontend Startup
```
▲ Next.js 14.2.33
- Local: http://localhost:3000
- Environments: .env.local
✓ Starting...
```

### Validation Output
```
🎉 VALIDATION PASSED!
✅ All environment variables are properly configured
🚀 System is ready for deployment!
```

## 📞 Support

If you encounter issues:
1. **Run setup script**: `./setup-dev.sh`
2. **Check validation**: `python3 validate-env.py`
3. **Verify file existence**: `ls -la backend/.env frontend/.env.local`
4. **Check service logs** for specific error messages