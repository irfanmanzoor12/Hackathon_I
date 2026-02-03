/**
 * better-auth configuration
 * Supports email/password and Google OAuth
 */

import { betterAuth } from 'better-auth';
import { Pool } from 'pg';
import dotenv from 'dotenv';

dotenv.config();

// Create PostgreSQL connection pool for Neon
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

export const auth = betterAuth({
  database: pool,

  // Email and password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    minPasswordLength: 8
  },

  // Social authentication - Google
  socialProviders: {
    google: {
      clientId: process.env.GOOGLE_CLIENT_ID,
      clientSecret: process.env.GOOGLE_CLIENT_SECRET
    }
  },

  // Session configuration
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24
  },

  // Base URL configuration
  baseURL: process.env.AUTH_BASE_URL || 'http://localhost:3001',

  // Secret for signing tokens
  secret: process.env.AUTH_SECRET || process.env.SECRET_KEY,

  // Trust proxy for production
  trustedOrigins: [
    process.env.FRONTEND_URL || 'http://localhost:3000',
    process.env.BACKEND_URL || 'http://localhost:8000'
  ]
});
