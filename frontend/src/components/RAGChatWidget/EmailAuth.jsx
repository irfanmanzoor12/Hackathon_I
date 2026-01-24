/**
 * Email/Password Authentication Component
 */

import React, { useState } from 'react';
import { motion } from 'framer-motion';
import './EmailAuth.css';
import { chatAPI } from '../../api/chatAPI';

const EmailAuth = ({ onSuccess, onBackToGoogle }) => {
  const [isLogin, setIsLogin] = useState(true);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    software_background: '',
    hardware_background: ''
  });
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const handleChange = (e) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
    setError('');
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError('');

    try {
      const endpoint = isLogin ? '/auth/login' : '/auth/register';
      const payload = isLogin
        ? { email: formData.email, password: formData.password }
        : formData;

      const response = await fetch(`${chatAPI.API_BASE}${endpoint}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Authentication failed');
      }

      // Save token
      localStorage.setItem('rag_token', data.access_token);
      onSuccess(data.user);

    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="email-auth">
      <div className="auth-header">
        <button className="back-btn" onClick={onBackToGoogle}>
          ← Back
        </button>
        <h3>{isLogin ? 'Login' : 'Create Account'}</h3>
      </div>

      <form onSubmit={handleSubmit} className="auth-form">
        {!isLogin && (
          <div className="form-group">
            <label>Name</label>
            <input
              type="text"
              name="name"
              value={formData.name}
              onChange={handleChange}
              required
              placeholder="Your name"
            />
          </div>
        )}

        <div className="form-group">
          <label>Email</label>
          <input
            type="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
            placeholder="your@email.com"
          />
        </div>

        <div className="form-group">
          <label>Password</label>
          <input
            type="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
            placeholder="••••••••"
            minLength="6"
          />
        </div>

        {!isLogin && (
          <>
            <div className="form-group">
              <label>Software Background (Optional)</label>
              <select
                name="software_background"
                value={formData.software_background}
                onChange={handleChange}
              >
                <option value="">Select level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className="form-group">
              <label>Hardware Background (Optional)</label>
              <select
                name="hardware_background"
                value={formData.hardware_background}
                onChange={handleChange}
              >
                <option value="">Select level</option>
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>
          </>
        )}

        {error && (
          <motion.div
            className="error-message"
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
          >
            {error}
          </motion.div>
        )}

        <motion.button
          type="submit"
          className="submit-btn"
          disabled={isLoading}
          whileHover={{ scale: 1.02 }}
          whileTap={{ scale: 0.98 }}
        >
          {isLoading ? 'Please wait...' : (isLogin ? 'Login' : 'Create Account')}
        </motion.button>

        <div className="toggle-mode">
          {isLogin ? "Don't have an account? " : "Already have an account? "}
          <button
            type="button"
            onClick={() => {
              setIsLogin(!isLogin);
              setError('');
            }}
            className="toggle-link"
          >
            {isLogin ? 'Sign up' : 'Login'}
          </button>
        </div>
      </form>
    </div>
  );
};

export default EmailAuth;
