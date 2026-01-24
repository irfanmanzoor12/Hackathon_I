/**
 * better-auth Client for Frontend
 * Handles authentication with background questions at signup
 */

const AUTH_BASE = typeof window !== 'undefined'
  ? (window.ENV?.AUTH_URL || 'http://localhost:3001')
  : 'http://localhost:3001';

class AuthClient {
  constructor() {
    this.baseUrl = AUTH_BASE;
  }

  /**
   * Sign up with email/password and background info
   */
  async signUp({ email, password, name, softwareBackground, hardwareBackground }) {
    const response = await fetch(`${this.baseUrl}/api/auth/sign-up/email`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({
        email,
        password,
        name,
        software_background: softwareBackground,
        hardware_background: hardwareBackground
      })
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      throw new Error(error.message || 'Sign up failed');
    }

    const data = await response.json();
    this.storeSession(data);
    return data;
  }

  /**
   * Sign in with email/password
   */
  async signIn({ email, password }) {
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
    this.storeSession(data);
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
