/**
 * RAG Chat Widget - Agentic Animated Chatbot
 * Physical AI & Humanoid Robotics - Chapter 1
 * Updated to use better-auth for authentication
 */

import React, { useState, useEffect, useRef } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import './RAGChatWidget.css';
import { chatAPI } from '../../api/chatAPI';
import { authClient } from '../../api/authClient';
import AgenticAvatar from './AgenticAvatar';
import AuthModal from '../AuthModal';

const RAGChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState(null);
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [botState, setBotState] = useState('idle'); // idle, listening, thinking, responding
  const [sessionId, setSessionId] = useState(null);
  const [selectedText, setSelectedText] = useState('');
  const [showAuthModal, setShowAuthModal] = useState(false);

  const messagesEndRef = useRef(null);
  const chatInputRef = useRef(null);

  // Check authentication on mount
  useEffect(() => {
    const token = localStorage.getItem('rag_token');
    if (token) {
      verifyToken(token);
    }

    // Listen for text selection
    document.addEventListener('mouseup', handleTextSelection);
    return () => document.removeEventListener('mouseup', handleTextSelection);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const verifyToken = async (token) => {
    try {
      // Try better-auth session first
      const session = await authClient.getSession();
      if (session?.user) {
        setIsAuthenticated(true);
        setUser(session.user);
        addSystemMessage('Welcome back! I\'m your Physical AI tutor. Ask me anything about Chapter 1!');
        return;
      }

      // Fall back to legacy token verification
      const userData = await chatAPI.verifyToken(token);
      setIsAuthenticated(true);
      setUser(userData);
      addSystemMessage('Welcome back! I\'m your Physical AI tutor. Ask me anything about Chapter 1!');
    } catch (error) {
      localStorage.removeItem('rag_token');
      setIsAuthenticated(false);
    }
  };

  const handleTextSelection = () => {
    const selection = window.getSelection();
    const text = selection.toString().trim();

    if (text && text.length > 10 && text.length < 1000) {
      setSelectedText(text);
      if (!isOpen) setIsOpen(true);
      addSystemMessage(`📝 Text selected! I can help explain: "${text.substring(0, 50)}..."`);
    }
  };

  const handleOpenAuthModal = () => {
    setShowAuthModal(true);
  };

  const handleAuthSuccess = () => {
    const storedUser = authClient.getStoredUser();
    setIsAuthenticated(true);
    setUser(storedUser);
    setShowAuthModal(false);
    addSystemMessage(`Welcome ${storedUser?.name || 'back'}! I'm your Physical AI tutor. Ask me anything about Chapter 1!`);
  };

  const handleLogout = async () => {
    await authClient.signOut();
    setIsAuthenticated(false);
    setUser(null);
    setMessages([]);
    setSessionId(null);
  };

  const addSystemMessage = (content) => {
    setMessages(prev => [...prev, {
      role: 'system',
      content,
      timestamp: new Date()
    }]);
  };

  const sendMessage = async (action = null) => {
    if (!inputMessage.trim() && !selectedText) return;

    const userMessage = inputMessage.trim() || 'Explain this selected text';

    // Add user message
    setMessages(prev => [...prev, {
      role: 'user',
      content: userMessage,
      timestamp: new Date()
    }]);

    setInputMessage('');
    setIsLoading(true);
    setBotState('thinking');

    try {
      const response = await chatAPI.sendMessage({
        message: userMessage,
        session_id: sessionId,
        selected_text: selectedText || null,
        action: action
      });

      // Set session ID
      if (!sessionId) {
        setSessionId(response.session_id);
      }

      // Add bot response
      setBotState('responding');
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: response.response,
        sources: response.sources,
        timestamp: new Date()
      }]);

      // Clear selected text
      setSelectedText('');

    } catch (error) {
      console.error('Chat error:', error);
      addSystemMessage('❌ Sorry, something went wrong. Please try again.');
    } finally {
      setIsLoading(false);
      setBotState('idle');
    }
  };

  const handlePersonalize = async () => {
    if (!inputMessage.trim() && !selectedText) return;
    await sendMessage('personalize');
  };

  const handleTranslate = async () => {
    if (!inputMessage.trim() && !selectedText) return;
    await sendMessage('translate');
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Widget toggle button
  const toggleWidget = () => {
    setIsOpen(!isOpen);
    if (!isOpen && botState === 'idle') {
      setBotState('listening');
      setTimeout(() => setBotState('idle'), 2000);
    }
  };

  return (
    <>
      {/* Floating Toggle Button */}
      <motion.button
        className="rag-chat-toggle"
        onClick={toggleWidget}
        whileHover={{ scale: 1.1 }}
        whileTap={{ scale: 0.95 }}
        animate={{
          boxShadow: isOpen
            ? '0 0 0 4px rgba(59, 130, 246, 0.3)'
            : '0 4px 20px rgba(0, 0, 0, 0.15)'
        }}
      >
        <AgenticAvatar state={botState} size="small" />
        {!isOpen && (
          <motion.span
            className="notification-badge"
            initial={{ scale: 0 }}
            animate={{ scale: 1 }}
          >
            AI
          </motion.span>
        )}
      </motion.button>

      {/* Chat Widget Panel */}
      <AnimatePresence>
        {isOpen && (
          <motion.div
            className="rag-chat-widget"
            initial={{ opacity: 0, y: 20, scale: 0.95 }}
            animate={{ opacity: 1, y: 0, scale: 1 }}
            exit={{ opacity: 0, y: 20, scale: 0.95 }}
            transition={{ duration: 0.3 }}
          >
            {/* Header */}
            <div className="rag-chat-header">
              <div className="header-content">
                <AgenticAvatar state={botState} size="tiny" />
                <div>
                  <h3>Physical AI Tutor</h3>
                  <span className="status-indicator">
                    {botState === 'idle' && '● Online'}
                    {botState === 'listening' && '👂 Listening...'}
                    {botState === 'thinking' && '🤔 Thinking...'}
                    {botState === 'responding' && '💬 Responding...'}
                  </span>
                </div>
              </div>
              <button className="close-btn" onClick={toggleWidget}>✕</button>
            </div>

            {/* Auth Section */}
            {!isAuthenticated ? (
              <div className="auth-section">
                <motion.div
                  className="auth-prompt"
                  initial={{ opacity: 0, y: 10 }}
                  animate={{ opacity: 1, y: 0 }}
                >
                  <AgenticAvatar state="idle" size="medium" />
                  <h3>Welcome to Physical AI!</h3>
                  <p>Sign in to personalize your learning experience and unlock all features.</p>

                  <motion.button
                    className="auth-signin-btn"
                    onClick={handleOpenAuthModal}
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                  >
                    Sign In / Sign Up
                  </motion.button>

                  <p className="auth-hint">
                    We'll ask about your background to personalize content
                  </p>
                </motion.div>

                <AuthModal
                  isOpen={showAuthModal}
                  onClose={() => setShowAuthModal(false)}
                  onSuccess={handleAuthSuccess}
                />
              </div>
            ) : (
              <>
                {/* Messages Area */}
                <div className="rag-chat-messages">
                  {messages.map((msg, idx) => (
                    <motion.div
                      key={idx}
                      className={`message message-${msg.role}`}
                      initial={{ opacity: 0, y: 10 }}
                      animate={{ opacity: 1, y: 0 }}
                      transition={{ delay: idx * 0.1 }}
                    >
                      {msg.role === 'assistant' && (
                        <AgenticAvatar state="idle" size="tiny" />
                      )}
                      <div className="message-content">
                        <div dangerouslySetInnerHTML={{ __html: formatMessage(msg.content) }} />
                        {msg.sources && msg.sources.length > 0 && (
                          <div className="message-sources">
                            <small>📚 Sources:</small>
                            {msg.sources.map((src, i) => (
                              <small key={i}>{src.section}</small>
                            ))}
                          </div>
                        )}
                      </div>
                      {msg.role === 'user' && (
                        <div className="user-avatar">{user?.name?.[0] || 'U'}</div>
                      )}
                    </motion.div>
                  ))}

                  {isLoading && (
                    <motion.div
                      className="message message-assistant"
                      initial={{ opacity: 0 }}
                      animate={{ opacity: 1 }}
                    >
                      <AgenticAvatar state={botState} size="tiny" />
                      <div className="typing-indicator">
                        <span></span><span></span><span></span>
                      </div>
                    </motion.div>
                  )}

                  <div ref={messagesEndRef} />
                </div>

                {/* Selected Text Preview */}
                {selectedText && (
                  <motion.div
                    className="selected-text-preview"
                    initial={{ opacity: 0, height: 0 }}
                    animate={{ opacity: 1, height: 'auto' }}
                  >
                    <span>📝 Selected: {selectedText.substring(0, 80)}...</span>
                    <button onClick={() => setSelectedText('')}>✕</button>
                  </motion.div>
                )}

                {/* Action Buttons */}
                <div className="action-buttons">
                  <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    onClick={handlePersonalize}
                    disabled={(!inputMessage.trim() && !selectedText) || isLoading}
                  >
                    ✨ Personalize
                  </motion.button>
                  <motion.button
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                    onClick={handleTranslate}
                    disabled={(!inputMessage.trim() && !selectedText) || isLoading}
                  >
                    🌐 Translate
                  </motion.button>
                </div>

                {/* Input Area */}
                <div className="rag-chat-input">
                  <textarea
                    ref={chatInputRef}
                    value={inputMessage}
                    onChange={(e) => setInputMessage(e.target.value)}
                    onKeyPress={handleKeyPress}
                    placeholder={selectedText ? "Ask about selected text..." : "Ask about Physical AI, ROS 2, robotics..."}
                    rows={2}
                    disabled={isLoading}
                  />
                  <motion.button
                    onClick={() => sendMessage()}
                    disabled={(!inputMessage.trim() && !selectedText) || isLoading}
                    whileHover={{ scale: 1.1 }}
                    whileTap={{ scale: 0.9 }}
                  >
                    ➤
                  </motion.button>
                </div>

                {/* User Info */}
                <div className="user-info">
                  <span>{user?.name}</span>
                  <button onClick={handleLogout}>Logout</button>
                </div>
              </>
            )}
          </motion.div>
        )}
      </AnimatePresence>
    </>
  );
};

// Format message content (basic markdown support)
const formatMessage = (content) => {
  return content
    .replace(/```([\s\S]*?)```/g, '<pre><code>$1</code></pre>')
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
    .replace(/\n/g, '<br/>');
};

export default RAGChatWidget;
