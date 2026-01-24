# Docusaurus Integration Guide
## RAG Chatbot Widget for Chapter 1: Physical AI & Humanoid Robotics

This guide explains how to integrate the RAG Chatbot widget into your Docusaurus book project.

---

## Prerequisites

1. Docusaurus site (v2+)
2. React support enabled
3. Backend API running (FastAPI)
4. Environment variables configured

---

## Installation Steps

### 1. Install Required Dependencies

```bash
npm install framer-motion
# or
yarn add framer-motion
```

### 2. Copy Widget Components

Copy the entire `src/components/RAGChatWidget` folder to your Docusaurus site's `src/components/` directory:

```
your-docusaurus-site/
├── src/
│   └── components/
│       └── RAGChatWidget/
│           ├── RAGChatWidget.jsx
│           ├── RAGChatWidget.css
│           ├── AgenticAvatar.jsx
│           └── AgenticAvatar.css
```

### 3. Copy API Integration

Copy `src/api/chatAPI.js` to `src/api/` in your Docusaurus site.

### 4. Configure Environment Variables

Create or update `docusaurus.config.js`:

```javascript
module.exports = {
  // ... other config
  customFields: {
    REACT_APP_API_URL: process.env.REACT_APP_API_URL || 'http://localhost:8000',
  },
};
```

Create `.env` file in your Docusaurus root:

```bash
REACT_APP_API_URL=http://localhost:8000
# Production:
# REACT_APP_API_URL=https://your-backend-domain.com
```

### 5. Handle OAuth Callback

Create a new page `src/pages/auth/callback.js`:

```javascript
import React, { useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';

export default function AuthCallback() {
  const history = useHistory();
  const location = useLocation();

  useEffect(() => {
    // Extract token from URL
    const params = new URLSearchParams(location.search);
    const token = params.get('token');

    if (token) {
      // Save token
      localStorage.setItem('rag_token', token);

      // Redirect to home or previous page
      const returnTo = localStorage.getItem('auth_return_to') || '/';
      localStorage.removeItem('auth_return_to');
      history.push(returnTo);
    } else {
      // Handle error
      const error = params.get('message');
      console.error('Auth error:', error);
      history.push('/');
    }
  }, [history, location]);

  return (
    <div style={{ padding: '50px', textAlign: 'center' }}>
      <h2>Authenticating...</h2>
      <p>Please wait while we complete the login process.</p>
    </div>
  );
}
```

---

## Integration Methods

### Method 1: Global Widget (Recommended)

Add the widget globally so it appears on all pages.

**Edit `src/theme/Root.js`** (create if doesn't exist):

```javascript
import React from 'react';
import RAGChatWidget from '@site/src/components/RAGChatWidget/RAGChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <RAGChatWidget />
    </>
  );
}
```

### Method 2: Specific Pages Only

Import and use in specific MDX files:

**In `docs/chapter-1-physical-ai.mdx`:**

```mdx
---
title: Chapter 1 - Physical AI & Humanoid Robotics
---

import RAGChatWidget from '@site/src/components/RAGChatWidget/RAGChatWidget';

# Chapter 1: Physical AI & Humanoid Robotics

Welcome to Physical AI! This quarter focuses on bridging the gap between digital intelligence and physical embodiment.

## Overview

Physical AI represents the intersection of artificial intelligence and robotics...

<RAGChatWidget />
```

### Method 3: Layout Wrapper

Create a custom layout for specific sections:

**Create `src/components/ChapterLayout.jsx`:**

```javascript
import React from 'react';
import RAGChatWidget from './RAGChatWidget/RAGChatWidget';

export default function ChapterLayout({ children }) {
  return (
    <div className="chapter-layout">
      {children}
      <RAGChatWidget />
    </div>
  );
}
```

**Use in MDX:**

```mdx
import ChapterLayout from '@site/src/components/ChapterLayout';

<ChapterLayout>

# Your content here...

</ChapterLayout>
```

---

## Customization

### Styling

Override default styles by creating a custom CSS file:

```css
/* src/css/custom.css */

/* Change widget colors */
.rag-chat-widget {
  --primary-color: #3B82F6;
  --secondary-color: #1D4ED8;
  --accent-color: #60A5FA;
}

/* Adjust position for mobile */
@media (max-width: 768px) {
  .rag-chat-toggle {
    bottom: 80px; /* Account for Docusaurus mobile nav */
  }
}
```

### Widget Props (Optional Enhancement)

You can modify `RAGChatWidget.jsx` to accept props:

```javascript
const RAGChatWidget = ({
  position = 'bottom-right',
  theme = 'blue',
  defaultOpen = false
}) => {
  // ... implementation
};
```

---

## Testing the Integration

### 1. Start Backend

```bash
cd backend
uv pip install -r requirements.txt
uvicorn app.main:app --reload
```

### 2. Start Docusaurus

```bash
cd your-docusaurus-site
npm start
```

### 3. Test Features

1. **Widget appears:** Check bottom-right corner for floating button
2. **Authentication:** Click widget → Sign in with Google
3. **Text selection:** Select text from Chapter 1 content
4. **Chat:** Ask questions about ROS 2, Gazebo, Isaac
5. **Personalization:** Test "Personalize" button
6. **Translation:** Test "Translate to Urdu" button

---

## Production Deployment

### Backend Deployment

Deploy FastAPI backend to:
- **Railway**: `railway up`
- **Render**: Connect GitHub repo
- **Google Cloud Run**: `gcloud run deploy`
- **AWS Fargate**: Docker container

Update CORS settings in backend:

```python
# app/config.py
CORS_ORIGINS = [
    "https://your-docusaurus-domain.com",
    "https://www.your-docusaurus-domain.com"
]
```

### Frontend Configuration

Update production environment variable:

```bash
# .env.production
REACT_APP_API_URL=https://your-backend-domain.com
```

### Google OAuth Configuration

Add production redirect URIs in Google Cloud Console:

```
https://your-backend-domain.com/auth/google/callback
```

---

## Troubleshooting

### Widget Doesn't Appear

1. Check console for errors
2. Verify `framer-motion` is installed
3. Ensure `Root.js` is properly configured
4. Clear browser cache

### Authentication Fails

1. Verify Google OAuth credentials
2. Check redirect URI matches exactly
3. Ensure backend is running
4. Check CORS configuration

### Chat Not Working

1. Verify backend API is accessible
2. Check JWT token in localStorage
3. Verify Qdrant and OpenAI credentials
4. Check network tab for API errors

### Text Selection Not Working

1. Ensure event listener is attached
2. Check for conflicting JavaScript
3. Verify selection length (10-1000 chars)

---

## Chapter 1 Content Indexing

To enable RAG functionality, you need to index Chapter 1 content into Qdrant.

### Create Indexing Script

**`backend/scripts/index_chapter1.py`:**

```python
import asyncio
from openai import AsyncOpenAI
from qdrant_client import QdrantClient
from app.config import settings
from app.database.qdrant import upsert_documents

# Chapter 1 content sections
CHAPTER_1_CONTENT = [
    {
        "content": "Physical AI represents AI systems that interact with and understand the physical world...",
        "metadata": {"section": "Introduction", "week": "1"},
    },
    {
        "content": "ROS 2 (Robot Operating System 2) is the middleware framework for robotics...",
        "metadata": {"section": "ROS 2 Basics", "week": "2-4"},
    },
    # Add all Chapter 1 sections...
]

async def generate_embeddings(texts):
    client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
    response = await client.embeddings.create(
        model=settings.OPENAI_EMBEDDING_MODEL,
        input=texts
    )
    return [item.embedding for item in response.data]

async def index_content():
    print("Generating embeddings...")
    texts = [item["content"] for item in CHAPTER_1_CONTENT]
    embeddings = await generate_embeddings(texts)

    print("Upserting to Qdrant...")
    upsert_documents(CHAPTER_1_CONTENT, embeddings)

    print("✅ Chapter 1 content indexed successfully!")

if __name__ == "__main__":
    asyncio.run(index_content())
```

Run:

```bash
python backend/scripts/index_chapter1.py
```

---

## Advanced Features

### Add Profile Setup Modal

Prompt users to set their IT background on first use:

```javascript
// Add to RAGChatWidget.jsx
const [showProfileSetup, setShowProfileSetup] = useState(false);

// After authentication, check if profile is complete
useEffect(() => {
  if (user && !user.software_background) {
    setShowProfileSetup(true);
  }
}, [user]);
```

### Add Voice Input

Integrate Web Speech API:

```javascript
const startVoiceInput = () => {
  const recognition = new window.webkitSpeechRecognition();
  recognition.onresult = (event) => {
    const transcript = event.results[0][0].transcript;
    setInputMessage(transcript);
  };
  recognition.start();
};
```

### Add Conversation Export

Allow users to download chat history:

```javascript
const exportConversation = () => {
  const text = messages.map(m => `${m.role}: ${m.content}`).join('\n\n');
  const blob = new Blob([text], { type: 'text/plain' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'chapter1-conversation.txt';
  a.click();
};
```

---

## Support

For issues or questions:

1. Check backend logs: `uvicorn app.main:app --log-level debug`
2. Check browser console for frontend errors
3. Verify all environment variables are set
4. Test API endpoints directly with curl/Postman

---

**You're all set!** The RAG chatbot should now be fully integrated into your Docusaurus Chapter 1 content.
