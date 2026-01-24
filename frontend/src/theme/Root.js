/**
 * Root wrapper for Docusaurus - adds global components
 * This wraps the entire Docusaurus app
 */

import React from 'react';
import RAGChatWidget from '../components/RAGChatWidget/RAGChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <RAGChatWidget />
    </>
  );
}
