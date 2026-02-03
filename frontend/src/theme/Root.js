/**
 * Root wrapper for Docusaurus - adds global components
 * This wraps the entire Docusaurus app
 */

import React, { useState, useEffect, Suspense } from 'react';

const RAGChatWidget = React.lazy(() => import('../components/RAGChatWidget/RAGChatWidget'));

export default function Root({children}) {
  const [showWidget, setShowWidget] = useState(false);

  useEffect(() => {
    // Delay widget load to ensure page renders first
    const timer = setTimeout(() => setShowWidget(true), 100);
    return () => clearTimeout(timer);
  }, []);

  return (
    <>
      {children}
      {showWidget && (
        <Suspense fallback={null}>
          <RAGChatWidget />
        </Suspense>
      )}
    </>
  );
}
