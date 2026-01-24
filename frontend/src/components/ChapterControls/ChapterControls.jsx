import React, { useState } from 'react';
import './ChapterControls.css';

const ChapterControls = ({ chapterContent, chapterTitle }) => {
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState(null);
  const [translatedContent, setTranslatedContent] = useState(null);

  const handlePersonalize = async () => {
    setIsPersonalizing(true);
    try {
      const response = await fetch(`${process.env.REACT_APP_API_URL || 'http://localhost:8000'}/chat/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('auth_token')}`,
        },
        body: JSON.stringify({
          content: chapterContent,
          chapter_title: chapterTitle,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setPersonalizedContent(data.personalized_content);

        // Show personalized version
        const banner = document.createElement('div');
        banner.className = 'personalization-banner';
        banner.innerHTML = `
          <h4>=Ú Personalized for Your Level</h4>
          <div>${data.personalized_content}</div>
          <button onclick="this.parentElement.remove()" style="margin-top: 10px;">Close</button>
        `;
        document.querySelector('article').prepend(banner);
      } else if (response.status === 401) {
        alert('Please sign in to use personalization features.');
      }
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleTranslate = async () => {
    setIsTranslating(true);
    try {
      const response = await fetch(`${process.env.REACT_APP_API_URL || 'http://localhost:8000'}/chat/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('auth_token')}`,
        },
        body: JSON.stringify({
          content: chapterContent,
          target_language: 'urdu',
          chapter_title: chapterTitle,
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setTranslatedContent(data.translated_content);

        // Show translated version
        const banner = document.createElement('div');
        banner.className = 'translation-banner';
        banner.innerHTML = `
          <h4>< '1/H EÌº (In Urdu)</h4>
          <div style="direction: rtl; text-align: right;">${data.translated_content}</div>
          <button onclick="this.parentElement.remove()" style="margin-top: 10px;">Close / (F/ ©1Ìº</button>
        `;
        document.querySelector('article').prepend(banner);
      } else if (response.status === 401) {
        alert('Please sign in to use translation features.');
      }
    } catch (error) {
      console.error('Translation error:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div className="chapter-controls">
      <div className="chapter-controls-container">
        <button
          className="chapter-personalize-btn"
          onClick={handlePersonalize}
          disabled={isPersonalizing}
        >
          {isPersonalizing ? 'ó Personalizing...' : '<¯ Personalize for Me'}
        </button>

        <button
          className="chapter-translate-btn"
          onClick={handleTranslate}
          disabled={isTranslating}
        >
          {isTranslating ? 'ó Translating...' : '< Translate to Urdu'}
        </button>
      </div>

      <p className="chapter-controls-hint">
        =¡ Personalize content based on your background or translate to Urdu
      </p>
    </div>
  );
};

export default ChapterControls;
