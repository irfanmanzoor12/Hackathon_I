/**
 * AuthModal - Sign Up / Sign In with better-auth
 * Asks software/hardware background questions at signup for personalization
 */

import React, { useState } from 'react';
import { authClient } from '../../api/authClient';
import './AuthModal.css';

const SOFTWARE_LEVELS = [
  { value: 'beginner', label: 'Beginner', description: 'New to programming or just started learning' },
  { value: 'intermediate', label: 'Intermediate', description: 'Comfortable with Python, some experience with APIs' },
  { value: 'advanced', label: 'Advanced', description: 'Experienced developer, familiar with ML/AI concepts' }
];

const HARDWARE_LEVELS = [
  { value: 'beginner', label: 'Beginner', description: 'No robotics hardware experience' },
  { value: 'intermediate', label: 'Intermediate', description: 'Used Arduino, Raspberry Pi, or similar' },
  { value: 'advanced', label: 'Advanced', description: 'Experience with ROS, industrial robots, or Jetson' }
];

export default function AuthModal({ isOpen, onClose, onSuccess }) {
  const [mode, setMode] = useState('signin'); // 'signin', 'signup', 'background'
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  // Form data
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    softwareBackground: '',
    hardwareBackground: ''
  });

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
    setError(null);
  };

  const handleGoogleSignIn = () => {
    authClient.signInWithGoogle();
  };

  const handleEmailSignIn = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      await authClient.signIn({
        email: formData.email,
        password: formData.password
      });
      onSuccess?.();
      onClose();
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  const handleEmailSignUp = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      // Validate required fields
      if (!formData.email || !formData.password || !formData.name) {
        throw new Error('Please fill in all required fields');
      }
      if (formData.password.length < 8) {
        throw new Error('Password must be at least 8 characters');
      }

      // Move to background questions step
      setMode('background');
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  const handleBackgroundSubmit = async (e) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      if (!formData.softwareBackground || !formData.hardwareBackground) {
        throw new Error('Please select your experience levels');
      }

      await authClient.signUp({
        email: formData.email,
        password: formData.password,
        name: formData.name,
        softwareBackground: formData.softwareBackground,
        hardwareBackground: formData.hardwareBackground
      });

      onSuccess?.();
      onClose();
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="auth-modal-overlay" onClick={onClose}>
      <div className="auth-modal" onClick={e => e.stopPropagation()}>
        <button className="auth-modal__close" onClick={onClose}>
          &times;
        </button>

        {mode === 'background' ? (
          // Background Questions Step
          <div className="auth-modal__content">
            <h2 className="auth-modal__title">Tell Us About Your Background</h2>
            <p className="auth-modal__subtitle">
              This helps us personalize content for your experience level
            </p>

            <form onSubmit={handleBackgroundSubmit}>
              {error && <div className="auth-modal__error">{error}</div>}

              <div className="auth-modal__field">
                <label className="auth-modal__label">Software/Programming Experience</label>
                <div className="auth-modal__options">
                  {SOFTWARE_LEVELS.map(level => (
                    <label
                      key={level.value}
                      className={`auth-modal__option ${formData.softwareBackground === level.value ? 'selected' : ''}`}
                    >
                      <input
                        type="radio"
                        name="softwareBackground"
                        value={level.value}
                        checked={formData.softwareBackground === level.value}
                        onChange={handleInputChange}
                      />
                      <span className="auth-modal__option-content">
                        <strong>{level.label}</strong>
                        <small>{level.description}</small>
                      </span>
                    </label>
                  ))}
                </div>
              </div>

              <div className="auth-modal__field">
                <label className="auth-modal__label">Hardware/Robotics Experience</label>
                <div className="auth-modal__options">
                  {HARDWARE_LEVELS.map(level => (
                    <label
                      key={level.value}
                      className={`auth-modal__option ${formData.hardwareBackground === level.value ? 'selected' : ''}`}
                    >
                      <input
                        type="radio"
                        name="hardwareBackground"
                        value={level.value}
                        checked={formData.hardwareBackground === level.value}
                        onChange={handleInputChange}
                      />
                      <span className="auth-modal__option-content">
                        <strong>{level.label}</strong>
                        <small>{level.description}</small>
                      </span>
                    </label>
                  ))}
                </div>
              </div>

              <div className="auth-modal__actions">
                <button
                  type="button"
                  className="auth-modal__btn auth-modal__btn--secondary"
                  onClick={() => setMode('signup')}
                >
                  Back
                </button>
                <button
                  type="submit"
                  className="auth-modal__btn auth-modal__btn--primary"
                  disabled={isLoading}
                >
                  {isLoading ? 'Creating Account...' : 'Complete Sign Up'}
                </button>
              </div>
            </form>
          </div>
        ) : (
          // Sign In / Sign Up Step
          <div className="auth-modal__content">
            <h2 className="auth-modal__title">
              {mode === 'signin' ? 'Welcome Back' : 'Create Account'}
            </h2>
            <p className="auth-modal__subtitle">
              {mode === 'signin'
                ? 'Sign in to personalize your learning experience'
                : 'Join to unlock personalized content and translation'}
            </p>

            {error && <div className="auth-modal__error">{error}</div>}

            <button
              type="button"
              className="auth-modal__google-btn"
              onClick={handleGoogleSignIn}
            >
              <svg viewBox="0 0 24 24" width="20" height="20">
                <path fill="#4285F4" d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"/>
                <path fill="#34A853" d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"/>
                <path fill="#FBBC05" d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"/>
                <path fill="#EA4335" d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"/>
              </svg>
              Continue with Google
            </button>

            <div className="auth-modal__divider">
              <span>or</span>
            </div>

            <form onSubmit={mode === 'signin' ? handleEmailSignIn : handleEmailSignUp}>
              {mode === 'signup' && (
                <div className="auth-modal__field">
                  <label className="auth-modal__label">Full Name</label>
                  <input
                    type="text"
                    name="name"
                    value={formData.name}
                    onChange={handleInputChange}
                    placeholder="Enter your name"
                    className="auth-modal__input"
                    required
                  />
                </div>
              )}

              <div className="auth-modal__field">
                <label className="auth-modal__label">Email</label>
                <input
                  type="email"
                  name="email"
                  value={formData.email}
                  onChange={handleInputChange}
                  placeholder="Enter your email"
                  className="auth-modal__input"
                  required
                />
              </div>

              <div className="auth-modal__field">
                <label className="auth-modal__label">Password</label>
                <input
                  type="password"
                  name="password"
                  value={formData.password}
                  onChange={handleInputChange}
                  placeholder={mode === 'signup' ? 'Create a password (min 8 chars)' : 'Enter your password'}
                  className="auth-modal__input"
                  minLength={mode === 'signup' ? 8 : undefined}
                  required
                />
              </div>

              <button
                type="submit"
                className="auth-modal__btn auth-modal__btn--primary auth-modal__btn--full"
                disabled={isLoading}
              >
                {isLoading
                  ? 'Please wait...'
                  : mode === 'signin'
                    ? 'Sign In'
                    : 'Continue'}
              </button>
            </form>

            <div className="auth-modal__switch">
              {mode === 'signin' ? (
                <p>
                  Don't have an account?{' '}
                  <button type="button" onClick={() => setMode('signup')}>
                    Sign Up
                  </button>
                </p>
              ) : (
                <p>
                  Already have an account?{' '}
                  <button type="button" onClick={() => setMode('signin')}>
                    Sign In
                  </button>
                </p>
              )}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
