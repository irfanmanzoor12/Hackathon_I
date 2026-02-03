/**
 * Chat API Integration
 * Handles all backend communication for RAG chatbot
 */

const API_BASE = typeof window !== 'undefined'
  ? (window.ENV?.API_URL || 'http://localhost:8000')
  : 'http://localhost:8000';

class ChatAPI {
  constructor() {
    this.API_BASE = API_BASE;
  }

  /**
   * Get authorization headers with JWT token
   */
  getHeaders() {
    const token = localStorage.getItem('rag_token');
    return {
      'Content-Type': 'application/json',
      ...(token && { 'Authorization': `Bearer ${token}` })
    };
  }

  /**
   * Handle API response
   */
  async handleResponse(response) {
    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      throw new Error(error.detail || `HTTP ${response.status}: ${response.statusText}`);
    }
    return response.json();
  }

  /**
   * Verify JWT token
   */
  async verifyToken(token) {
    const response = await fetch(`${this.API_BASE}/auth/verify?token=${token}`, {
      method: 'GET'
    });

    if (!response.ok) {
      throw new Error('Token verification failed');
    }

    const data = await response.json();
    return data;
  }

  /**
   * Send chat message
   */
  async sendMessage({ message, session_id = null, selected_text = null, action = null }) {
    const response = await fetch(`${this.API_BASE}/chat/message`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        message,
        session_id,
        selected_text,
        action
      })
    });

    return this.handleResponse(response);
  }

  /**
   * Personalize content
   */
  async personalizeContent(message) {
    const response = await fetch(`${this.API_BASE}/chat/personalize`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({ message })
    });

    return this.handleResponse(response);
  }

  /**
   * Translate content to Urdu
   */
  async translateContent(message) {
    const response = await fetch(`${this.API_BASE}/chat/translate`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({ message })
    });

    return this.handleResponse(response);
  }

  /**
   * Explain code snippet
   */
  async explainCode(code, context = null) {
    const response = await fetch(`${this.API_BASE}/chat/explain-code`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        message: context || '',
        selected_text: code
      })
    });

    return this.handleResponse(response);
  }

  /**
   * Get user profile
   */
  async getProfile() {
    const response = await fetch(`${this.API_BASE}/chat/profile`, {
      method: 'GET',
      headers: this.getHeaders()
    });

    return this.handleResponse(response);
  }

  /**
   * Update user profile
   */
  async updateProfile({ software_background, hardware_background }) {
    const response = await fetch(`${this.API_BASE}/chat/profile`, {
      method: 'PUT',
      headers: this.getHeaders(),
      body: JSON.stringify({
        software_background,
        hardware_background
      })
    });

    return this.handleResponse(response);
  }

  /**
   * Logout user
   */
  async logout() {
    const response = await fetch(`${this.API_BASE}/auth/logout`, {
      method: 'POST',
      headers: this.getHeaders()
    });

    localStorage.removeItem('rag_token');
    return this.handleResponse(response);
  }
}

export const chatAPI = new ChatAPI();
export default chatAPI;
