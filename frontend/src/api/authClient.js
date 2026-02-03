/**
 * better-auth Client for Frontend
 * Handles authentication with background questions at signup
 */

const AUTH_BASE = typeof window !== 'undefined'
  ? (window.ENV?.AUTH_URL || 'http://localhost:3001')
  : 'http://localhost:3001';

const API_BASE = typeof window !== 'undefined'
  ? (window.ENV?.API_URL || 'http://localhost:8000')
  : 'http://localhost:8000';

class AuthClient {
  constructor() {
    this.baseUrl = AUTH_BASE;
    this.apiUrl = API_BASE;
  }

  /**
   * Sign up with email/password
   * Registers with both better-auth and backend for full functionality
   */
  async signUp({ email, password, name }) {
    // First register with better-auth
    const response = await fetch(`${this.baseUrl}/api/auth/sign-up/email`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({
        email,
        password,
        name
      })
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      throw new Error(error.message || 'Sign up failed');
    }

    const data = await response.json();

    // Also register with backend to get JWT for chat functionality
    try {
      const softwareBg = localStorage.getItem('user_software_background') || 'beginner';
      const hardwareBg = localStorage.getItem('user_hardware_background') || 'beginner';

      const backendResponse = await fetch(`${this.apiUrl}/auth/register`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email,
          password,
          name,
          software_background: softwareBg,
          hardware_background: hardwareBg
        })
      });

      if (backendResponse.ok) {
        const backendData = await backendResponse.json();
        // Store the backend JWT token for chat API
        if (backendData.access_token) {
          localStorage.setItem('rag_token', backendData.access_token);
        }
        if (backendData.user) {
          localStorage.setItem('rag_user', JSON.stringify(backendData.user));
        }
      }
    } catch (err) {
      console.warn('Backend registration failed, using better-auth session:', err);
      this.storeSession(data);
    }

    return data;
  }

  /**
   * Sign in with email/password
   * Authenticates with both better-auth and backend for full functionality
   */
  async signIn({ email, password }) {
    // First sign in with better-auth
    const response = await fetch(`${this.baseUrl}/api/auth/sign-in/email`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({ email, password })
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      throw new Error(error.message || 'Sign in failed');
    }

    const data = await response.json();

    // Also login with backend to get JWT for chat functionality
    try {
      const backendResponse = await fetch(`${this.apiUrl}/auth/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password })
      });

      if (backendResponse.ok) {
        const backendData = await backendResponse.json();
        // Store the backend JWT token for chat API
        if (backendData.access_token) {
          localStorage.setItem('rag_token', backendData.access_token);
        }
        if (backendData.user) {
          localStorage.setItem('rag_user', JSON.stringify(backendData.user));
        }
      }
    } catch (err) {
      console.warn('Backend login failed, using better-auth session:', err);
      this.storeSession(data);
    }

    return data;
  }

  /**
   * Sign in with Google
   */
  signInWithGoogle() {
    const callbackUrl = encodeURIComponent(window.location.origin + '/auth/callback');
    window.location.href = `${this.baseUrl}/api/auth/sign-in/social?provider=google&callbackURL=${callbackUrl}`;
  }

  /**
   * Sign out
   */
  async signOut() {
    try {
      await fetch(`${this.baseUrl}/api/auth/sign-out`, {
        method: 'POST',
        credentials: 'include'
      });
    } catch (error) {
      console.error('Sign out error:', error);
    }
    this.clearSession();
  }

  /**
   * Get current session
   */
  async getSession() {
    try {
      const response = await fetch(`${this.baseUrl}/api/auth/get-session`, {
        method: 'GET',
        credentials: 'include'
      });

      if (!response.ok) {
        return null;
      }

      return await response.json();
    } catch (error) {
      console.error('Get session error:', error);
      return null;
    }
  }

  /**
   * Update user background info
   */
  async updateBackground({ softwareBackground, hardwareBackground }) {
    const response = await fetch(`${this.baseUrl}/api/user/background`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({
        software_background: softwareBackground,
        hardware_background: hardwareBackground
      })
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      throw new Error(error.message || 'Failed to update background');
    }

    return await response.json();
  }

  /**
   * Get user profile
   */
  async getProfile() {
    const response = await fetch(`${this.baseUrl}/api/user/profile`, {
      method: 'GET',
      credentials: 'include'
    });

    if (!response.ok) {
      throw new Error('Failed to fetch profile');
    }

    return await response.json();
  }

  /**
   * Store session in localStorage for persistence
   */
  storeSession(data) {
    if (data?.session?.token) {
      localStorage.setItem('rag_token', data.session.token);
    }
    if (data?.user) {
      localStorage.setItem('rag_user', JSON.stringify(data.user));
    }
  }

  /**
   * Clear session
   */
  clearSession() {
    localStorage.removeItem('rag_token');
    localStorage.removeItem('rag_user');
  }

  /**
   * Check if user is authenticated
   */
  isAuthenticated() {
    return !!localStorage.getItem('rag_token');
  }

  /**
   * Get stored user
   */
  getStoredUser() {
    const user = localStorage.getItem('rag_user');
    return user ? JSON.parse(user) : null;
  }
}

export const authClient = new AuthClient();
export default authClient;
