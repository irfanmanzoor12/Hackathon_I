/**
 * Auth Callback Page
 * Handles OAuth redirects from better-auth
 */

import React, { useEffect, useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import { authClient } from '../../api/authClient';

export default function AuthCallback() {
  const history = useHistory();
  const location = useLocation();
  const [status, setStatus] = useState('processing');
  const [error, setError] = useState(null);

  useEffect(() => {
    const handleCallback = async () => {
      try {
        const params = new URLSearchParams(location.search);
        const token = params.get('token');
        const errorMsg = params.get('error');

        if (errorMsg) {
          setError(errorMsg);
          setStatus('error');
          setTimeout(() => history.push('/'), 3000);
          return;
        }

        if (token) {
          // Legacy token handling
          localStorage.setItem('rag_token', token);
          setStatus('success');
          setTimeout(() => history.push('/'), 1500);
          return;
        }

        // Check better-auth session
        const session = await authClient.getSession();
        if (session?.user) {
          authClient.storeSession({ user: session.user, session });
          setStatus('success');
          setTimeout(() => history.push('/'), 1500);
          return;
        }

        // No token or session found
        setError('Authentication failed. Please try again.');
        setStatus('error');
        setTimeout(() => history.push('/'), 3000);
      } catch (err) {
        console.error('Auth callback error:', err);
        setError(err.message);
        setStatus('error');
        setTimeout(() => history.push('/'), 3000);
      }
    };

    handleCallback();
  }, [history, location]);

  return (
    <div style={{
      display: 'flex',
      flexDirection: 'column',
      alignItems: 'center',
      justifyContent: 'center',
      minHeight: '60vh',
      padding: '2rem',
      textAlign: 'center'
    }}>
      {status === 'processing' && (
        <>
          <div style={{
            width: '48px',
            height: '48px',
            border: '4px solid #e2e8f0',
            borderTopColor: '#667eea',
            borderRadius: '50%',
            animation: 'spin 1s linear infinite'
          }} />
          <h2 style={{ marginTop: '1.5rem', color: '#334155' }}>
            Authenticating...
          </h2>
          <p style={{ color: '#64748b' }}>
            Please wait while we verify your credentials
          </p>
        </>
      )}

      {status === 'success' && (
        <>
          <div style={{
            width: '64px',
            height: '64px',
            background: 'linear-gradient(135deg, #22c55e 0%, #16a34a 100%)',
            borderRadius: '50%',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            color: 'white',
            fontSize: '2rem'
          }}>
            ✓
          </div>
          <h2 style={{ marginTop: '1.5rem', color: '#334155' }}>
            Welcome!
          </h2>
          <p style={{ color: '#64748b' }}>
            Redirecting you back to the course...
          </p>
        </>
      )}

      {status === 'error' && (
        <>
          <div style={{
            width: '64px',
            height: '64px',
            background: 'linear-gradient(135deg, #ef4444 0%, #dc2626 100%)',
            borderRadius: '50%',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            color: 'white',
            fontSize: '2rem'
          }}>
            ✕
          </div>
          <h2 style={{ marginTop: '1.5rem', color: '#334155' }}>
            Authentication Failed
          </h2>
          <p style={{ color: '#ef4444' }}>
            {error || 'Something went wrong. Please try again.'}
          </p>
          <p style={{ color: '#64748b', fontSize: '0.875rem' }}>
            Redirecting you back...
          </p>
        </>
      )}

      <style>{`
        @keyframes spin {
          to { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
}
