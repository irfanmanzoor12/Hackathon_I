/**
 * Authentication Service using better-auth
 * Handles signup/signin with user background questions
 */

import express from 'express';
import cors from 'cors';
import { auth } from './auth.js';
import { toNodeHandler } from 'better-auth/node';
import dotenv from 'dotenv';

dotenv.config();

const app = express();
const PORT = process.env.AUTH_PORT || 3001;

// CORS configuration
app.use(cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3000',
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
  allowedHeaders: ['Content-Type', 'Authorization']
}));

app.use(express.json());

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({ status: 'ok', service: 'auth' });
});

// Mount better-auth handler for all /api/auth/* routes
app.all('/api/auth/*', toNodeHandler(auth));

// Custom endpoint to update user background after signup
app.put('/api/user/background', async (req, res) => {
  try {
    const authHeader = req.headers.authorization;
    if (!authHeader) {
      return res.status(401).json({ error: 'No authorization header' });
    }

    // Verify session through better-auth
    const session = await auth.api.getSession({
      headers: req.headers
    });

    if (!session?.user) {
      return res.status(401).json({ error: 'Invalid session' });
    }

    const { software_background, hardware_background } = req.body;

    // Update user profile with background info
    await auth.api.updateUser({
      body: {
        software_background,
        hardware_background
      },
      headers: req.headers
    });

    res.json({
      success: true,
      message: 'Background updated successfully'
    });
  } catch (error) {
    console.error('Background update error:', error);
    res.status(500).json({ error: 'Failed to update background' });
  }
});

// Get user profile with background
app.get('/api/user/profile', async (req, res) => {
  try {
    const session = await auth.api.getSession({
      headers: req.headers
    });

    if (!session?.user) {
      return res.status(401).json({ error: 'Not authenticated' });
    }

    res.json({
      user: {
        id: session.user.id,
        email: session.user.email,
        name: session.user.name,
        image: session.user.image,
        software_background: session.user.software_background,
        hardware_background: session.user.hardware_background,
        createdAt: session.user.createdAt
      }
    });
  } catch (error) {
    console.error('Profile fetch error:', error);
    res.status(500).json({ error: 'Failed to fetch profile' });
  }
});

app.listen(PORT, () => {
  console.log(`Auth service running on http://localhost:${PORT}`);
  console.log(`better-auth endpoints available at /api/auth/*`);
});
