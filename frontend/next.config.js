/** @type {import('next').NextConfig} */
const nextConfig = {
  // Temporarily disable standalone for easier Docker build
  // output: 'standalone',
  experimental: {
    outputFileTracingRoot: require('path').join(__dirname, '../'),
  },
  transpilePackages: ['@web-teleop-robot/shared'],
  // Disable SWC minifier to avoid download issues during Docker build
  swcMinify: false,
  env: {
    NEXT_PUBLIC_API_URL: process.env.NEXT_PUBLIC_API_URL,
    NEXT_PUBLIC_WS_URL: process.env.NEXT_PUBLIC_WS_URL,
  },
  async rewrites() {
    return [
      {
        source: '/api/:path*',
        destination: `${process.env.NEXT_PUBLIC_API_URL || 'http://localhost:3001'}/api/:path*`,
      },
    ];
  },
};

module.exports = nextConfig;