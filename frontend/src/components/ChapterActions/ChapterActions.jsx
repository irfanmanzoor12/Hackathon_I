/**
 * ChapterActions - Personalize and Translate buttons for chapter content
 * Shows at the start of each chapter for logged-in users
 */

import React, { useState, useEffect, useCallback } from 'react';
import './ChapterActions.css';

const API_BASE = typeof window !== 'undefined'
  ? (window.ENV?.API_URL || 'http://localhost:8000')
  : 'http://localhost:8000';

export default function ChapterActions() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [translatedContent, setTranslatedContent] = useState(null);
  const [activeView, setActiveView] = useState('original');
  const [error, setError] = useState(null);

  // Check authentication status and listen for changes
  useEffect(() => {
    const checkAuth = () => {
      const token = localStorage.getItem('rag_token');
      setIsAuthenticated(!!token);
    };

    // Check on mount
    checkAuth();

    // Listen for storage changes (when user signs in/out)
    const handleStorageChange = (e) => {
      if (e.key === 'rag_token' || e.key === 'rag_user') {
        checkAuth();
      }
    };

    // Also check periodically for same-tab changes
    const interval = setInterval(checkAuth, 1000);

    window.addEventListener('storage', handleStorageChange);
    return () => {
      window.removeEventListener('storage', handleStorageChange);
      clearInterval(interval);
    };
  }, []);

  // Get auth headers
  const getHeaders = useCallback(() => {
    const token = localStorage.getItem('rag_token');
    return {
      'Content-Type': 'application/json',
      ...(token && { 'Authorization': `Bearer ${token}` })
    };
  }, []);

  // Extract chapter content from the page
  const getChapterContent = useCallback(() => {
    const article = document.querySelector('article');
    if (!article) return '';

    // Get text content, excluding our component
    const clone = article.cloneNode(true);
    const actionsDiv = clone.querySelector('.chapter-actions');
    if (actionsDiv) actionsDiv.remove();

    return clone.textContent?.trim().slice(0, 8000) || '';
  }, []);

  // Handle personalization
  const handlePersonalize = async () => {
    setError(null);
    setIsLoading(true);

    try {
      const content = getChapterContent();
      if (!content) {
        throw new Error('No chapter content found');
      }

      const response = await fetch(`${API_BASE}/chat/personalize`, {
        method: 'POST',
        headers: getHeaders(),
        body: JSON.stringify({ message: content })
      });

      if (!response.ok) {
        const err = await response.json().catch(() => ({}));
        throw new Error(err.detail || 'Personalization failed');
      }

      const data = await response.json();
      setPersonalizedContent(data.response || data.message);
      setActiveView('personalized');
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  // Handle translation
  const handleTranslate = async () => {
    setError(null);
    setIsLoading(true);

    try {
      const content = getChapterContent();
      if (!content) {
        throw new Error('No chapter content found');
      }

      const response = await fetch(`${API_BASE}/chat/translate`, {
        method: 'POST',
        headers: getHeaders(),
        body: JSON.stringify({ message: content })
      });

      if (!response.ok) {
        const err = await response.json().catch(() => ({}));
        throw new Error(err.detail || 'Translation failed');
      }

      const data = await response.json();
      setTranslatedContent(data.response || data.message);
      setActiveView('translated');
    } catch (err) {
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  };

  // Reset to original view
  const handleShowOriginal = () => {
    setActiveView('original');
    setError(null);
  };

  // Show sign-in prompt for unauthenticated users
  if (!isAuthenticated) {
    const handleSignIn = () => {
      // Try to open the chat widget which has the auth modal
      const chatToggle = document.querySelector('.rag-chat-toggle');
      if (chatToggle) {
        chatToggle.click();
      } else {
        // Fallback: redirect to auth service
        window.location.href = 'http://localhost:3001/api/auth/sign-in';
      }
    };

    return (
      <div className="chapter-actions chapter-actions--guest">
        <div className="chapter-actions__guest-message">
          <span className="chapter-actions__icon">🔐</span>
          <span>Sign in to personalize content or translate to Urdu</span>
          <button
            onClick={handleSignIn}
            className="chapter-actions__signin-btn"
          >
            Sign In
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className="chapter-actions">
      <div className="chapter-actions__header">
        <span className="chapter-actions__title">📚 Chapter Actions</span>
        <div className="chapter-actions__buttons">
          <button
            className={`chapter-actions__btn chapter-actions__btn--personalize ${activeView === 'personalized' ? 'active' : ''}`}
            onClick={handlePersonalize}
            disabled={isLoading}
            title="Adapt content to your background level"
          >
            {isLoading && activeView !== 'translated' ? (
              <span className="chapter-actions__spinner" />
            ) : (
              <span className="chapter-actions__btn-icon">✨</span>
            )}
            Personalize for Me
          </button>

          <button
            className={`chapter-actions__btn chapter-actions__btn--translate ${activeView === 'translated' ? 'active' : ''}`}
            onClick={handleTranslate}
            disabled={isLoading}
            title="Translate chapter to Urdu"
          >
            {isLoading && activeView === 'translated' ? (
              <span className="chapter-actions__spinner" />
            ) : (
              <span className="chapter-actions__btn-icon">🌐</span>
            )}
            اردو میں پڑھیں
          </button>

          {activeView !== 'original' && (
            <button
              className="chapter-actions__btn chapter-actions__btn--original"
              onClick={handleShowOriginal}
              disabled={isLoading}
            >
              <span className="chapter-actions__btn-icon">↩</span>
              Show Original
            </button>
          )}
        </div>
      </div>

      {error && (
        <div className="chapter-actions__error">
          <span>⚠️ {error}</span>
        </div>
      )}

      {activeView === 'personalized' && personalizedContent && (
        <div className="chapter-actions__content chapter-actions__content--personalized">
          <div className="chapter-actions__content-header">
            <span>✨ Personalized for Your Level</span>
          </div>
          <div className="chapter-actions__content-body">
            {personalizedContent}
          </div>
        </div>
      )}

      {activeView === 'translated' && translatedContent && (
        <div className="chapter-actions__content chapter-actions__content--translated" dir="rtl">
          <div className="chapter-actions__content-header">
            <span>🌐 اردو ترجمہ</span>
          </div>
          <div className="chapter-actions__content-body">
            {translatedContent}
          </div>
        </div>
      )}
    </div>
  );
}
