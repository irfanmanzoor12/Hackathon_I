# Physical AI RAG Chatbot - Complete System
## Chapter 1: Physical AI & Humanoid Robotics

A full-stack RAG (Retrieval-Augmented Generation) chatbot system with Google OAuth authentication, agentic animations, and multi-context conversation support for the Physical AI & Humanoid Robotics course.

---

## 🚀 Features

### Backend (FastAPI)
- ✅ Google OAuth 2.0 authentication
- ✅ JWT token management
- ✅ Neon Serverless Postgres for user data
- ✅ Qdrant Cloud vector store for Chapter 1 embeddings
- ✅ OpenAI GPT-4o for RAG responses
- ✅ MCP Context7 multi-turn conversation handler
- ✅ Reusable subagents (Personalizer, Code Explainer, Translator)
- ✅ UV package management

### Frontend (React/Docusaurus)
- ✅ Agentic animated chatbot avatar
- ✅ Eye movements, mouth animations, state indicators
- ✅ Text selection for contextual questions
- ✅ Personalization based on IT background
- ✅ Translation to Urdu
- ✅ Responsive design (mobile, tablet, desktop)
- ✅ Framer Motion animations

### Testing
- ✅ Playwright end-to-end tests
- ✅ Authentication flow testing
- ✅ Chat functionality testing
- ✅ Multi-browser support

---

## 📁 Project Structure

```
humanoid/
├── backend/
│   ├── app/
│   │   ├── main.py                 # FastAPI application
│   │   ├── config.py               # Configuration settings
│   │   ├── auth/
│   │   │   ├── routes.py           # Google OAuth routes
│   │   │   └── schemas.py          # Pydantic models
│   │   ├── chat/
│   │   │   ├── routes.py           # Chat endpoints
│   │   │   ├── rag_engine.py       # RAG + MCP Context7
│   │   │   └── subagents.py        # Personalizer, Translator, Code Explainer
│   │   └── database/
│   │       ├── postgres.py         # Neon Postgres integration
│   │       └── qdrant.py           # Qdrant vector store
│   ├── requirements.txt            # UV dependencies
│   └── .env.example                # Environment template
│
├── frontend/
│   ├── src/
│   │   ├── components/
│   │   │   └── RAGChatWidget/
│   │   │       ├── RAGChatWidget.jsx       # Main widget component
│   │   │       ├── RAGChatWidget.css       # Widget styles
│   │   │       ├── AgenticAvatar.jsx       # Animated avatar
│   │   │       └── AgenticAvatar.css       # Avatar animations
│   │   └── api/
│   │       └── chatAPI.js          # Backend API integration
│   └── DOCUSAURUS_INTEGRATION.md   # Integration guide
│
└── tests/
    └── playwright/
        ├── chatbot.spec.js         # E2E tests
        ├── playwright.config.js    # Playwright config
        └── package.json            # Test dependencies
```

---

## 🛠️ Setup Instructions

### Prerequisites

1. **Python 3.12+**
2. **Node.js 18+**
3. **UV Package Manager** (Install: `pip install uv`)
4. **Neon Postgres Account** (https://neon.tech)
5. **Qdrant Cloud Account** (https://cloud.qdrant.io)
6. **OpenAI API Key** (https://platform.openai.com)
7. **Google OAuth Credentials** (https://console.cloud.google.com)

---

## Backend Setup

### 1. Install UV Package Manager

```bash
pip install uv
```

### 2. Clone and Navigate

```bash
cd backend
```

### 3. Create Virtual Environment

```bash
uv venv
# Activate:
# Windows: .venv\Scripts\activate
# Unix: source .venv/bin/activate
```

### 4. Install Dependencies

```bash
uv pip install -r requirements.txt
```

### 5. Configure Environment Variables

Copy `.env.example` to `.env`:

```bash
cp .env.example .env
```

Edit `.env`:

```env
# Security
SECRET_KEY=your-super-secret-key-min-32-chars-CHANGE-THIS

# Google OAuth
GOOGLE_CLIENT_ID=your-google-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret
GOOGLE_REDIRECT_URI=http://localhost:8000/auth/google/callback

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-xxx.neon.tech/humanoid_rag?sslmode=require

# Qdrant Cloud
QDRANT_URL=https://xxx-xxx.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=sk-your-openai-api-key

# Frontend
FRONTEND_URL=http://localhost:3000
```

### 6. Google OAuth Setup

1. Go to [Google Cloud Console](https://console.cloud.google.com/apis/credentials)
2. Create a new OAuth 2.0 Client ID
3. Add authorized redirect URIs:
   - `http://localhost:8000/auth/google/callback`
   - (Production URI when deploying)
4. Copy Client ID and Secret to `.env`

### 7. Neon Postgres Setup

1. Create account at [neon.tech](https://neon.tech)
2. Create new project: `humanoid-rag`
3. Copy connection string to `.env` as `DATABASE_URL`
4. Tables will be created automatically on first run

### 8. Qdrant Cloud Setup

1. Create account at [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create cluster (Free tier available)
3. Copy cluster URL and API key to `.env`
4. Collection will be created automatically

### 9. Index Chapter 1 Content

Create `scripts/index_chapter1.py`:

```python
import asyncio
from openai import AsyncOpenAI
from app.config import settings
from app.database.qdrant import upsert_documents
import sys
sys.path.insert(0, '..')

# Chapter 1 content
CHAPTER_1_SECTIONS = [
    {
        "content": "Physical AI represents AI systems that interact with and understand the physical world through embodied intelligence.",
        "metadata": {"section": "Introduction", "week": "1"},
    },
    {
        "content": "ROS 2 (Robot Operating System 2) is the middleware framework that enables communication between distributed processes in robotics using nodes, topics, services, and actions.",
        "metadata": {"section": "ROS 2 Fundamentals", "week": "2-4"},
    },
    {
        "content": "Gazebo and Unity are physics simulation environments used to create digital twins of robotic systems for testing before deployment.",
        "metadata": {"section": "Digital Twins", "week": "5-7"},
    },
    {
        "content": "NVIDIA Isaac provides AI-powered perception, navigation, and manipulation capabilities for robotics through Isaac Sim and Isaac ROS.",
        "metadata": {"section": "NVIDIA Isaac", "week": "8-10"},
    },
    {
        "content": "Vision-Language-Action (VLA) models combine multimodal AI with robotics control, enabling natural language commands and cognitive planning.",
        "metadata": {"section": "VLA Integration", "week": "11-13"},
    },
    # Add more sections...
]

async def generate_embeddings(texts):
    client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
    response = await client.embeddings.create(
        model=settings.OPENAI_EMBEDDING_MODEL,
        input=texts
    )
    return [item.embedding for item in response.data]

async def main():
    print("📚 Indexing Chapter 1 content...")
    texts = [s["content"] for s in CHAPTER_1_SECTIONS]
    embeddings = await generate_embeddings(texts)

    upsert_documents(CHAPTER_1_SECTIONS, embeddings)
    print(f"✅ Indexed {len(CHAPTER_1_SECTIONS)} sections!")

if __name__ == "__main__":
    asyncio.run(main())
```

Run:

```bash
python scripts/index_chapter1.py
```

### 10. Start Backend

```bash
uvicorn app.main:app --reload
```

Backend runs at: **http://localhost:8000**

API docs at: **http://localhost:8000/docs**

---

## Frontend Setup

### 1. Install Dependencies

```bash
cd frontend
npm install framer-motion
```

### 2. Configure Environment

Create `.env`:

```env
REACT_APP_API_URL=http://localhost:8000
```

### 3. Copy Components to Docusaurus

Copy the following to your Docusaurus site:

```bash
# Copy components
cp -r src/components/RAGChatWidget your-docusaurus-site/src/components/

# Copy API
cp -r src/api your-docusaurus-site/src/
```

### 4. Create OAuth Callback Page

**`your-docusaurus-site/src/pages/auth/callback.js`:**

```javascript
import React, { useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';

export default function AuthCallback() {
  const history = useHistory();
  const location = useLocation();

  useEffect(() => {
    const params = new URLSearchParams(location.search);
    const token = params.get('token');

    if (token) {
      localStorage.setItem('rag_token', token);
      history.push('/');
    } else {
      history.push('/');
    }
  }, [history, location]);

  return (
    <div style={{ padding: '50px', textAlign: 'center' }}>
      <h2>Authenticating...</h2>
    </div>
  );
}
```

### 5. Add Global Widget

**`your-docusaurus-site/src/theme/Root.js`:**

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

### 6. Start Docusaurus

```bash
cd your-docusaurus-site
npm start
```

Frontend runs at: **http://localhost:3000**

---

## Testing Setup

### 1. Install Playwright

```bash
cd tests/playwright
npm install
npx playwright install --with-deps
```

### 2. Run Tests

```bash
# All tests
npm test

# Specific browser
npm run test:chromium

# Headed mode
npm run test:headed

# Debug mode
npm run test:debug

# UI mode
npm run test:ui
```

### 3. View Reports

```bash
npm run report
```

---

## 🧪 Testing the System

### Test Authentication

1. Open http://localhost:3000
2. Click chatbot toggle (bottom-right)
3. Click "Sign in with Google"
4. Complete OAuth flow
5. You should be redirected back with token

### Test Chat

1. Type: "What is ROS 2?"
2. Press Enter or click send button
3. Avatar should animate (thinking → responding)
4. Response should appear with sources

### Test Text Selection

1. Select text from Chapter 1 content (10-1000 chars)
2. Widget should open with selected text preview
3. Type question about selected text
4. Response should focus on selected content

### Test Personalization

1. Click "Personalize" button
2. Response should adapt to your IT background
3. (First set your background in profile)

### Test Translation

1. Type a message
2. Click "Translate" button
3. Response should be in Urdu

---

## 🚢 Production Deployment

### Backend Deployment

#### Option 1: Railway

```bash
# Install Railway CLI
npm install -g railway

# Login
railway login

# Initialize
railway init

# Deploy
railway up
```

#### Option 2: Render

1. Connect GitHub repo
2. Select `backend` directory
3. Set build command: `uv pip install -r requirements.txt`
4. Set start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables

#### Option 3: Google Cloud Run

```bash
# Build Docker image
docker build -t gcr.io/PROJECT_ID/rag-backend .

# Push
docker push gcr.io/PROJECT_ID/rag-backend

# Deploy
gcloud run deploy rag-backend \
  --image gcr.io/PROJECT_ID/rag-backend \
  --platform managed \
  --region us-central1 \
  --allow-unauthenticated
```

### Frontend Deployment

Deploy Docusaurus to:
- **Vercel**: Connect GitHub → Deploy
- **Netlify**: Connect GitHub → Deploy
- **GitHub Pages**: `npm run deploy`

### Update Environment Variables

Production `.env`:

```env
DATABASE_URL=your-production-neon-url
QDRANT_URL=your-qdrant-cluster-url
OPENAI_API_KEY=your-openai-key
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-secret
GOOGLE_REDIRECT_URI=https://your-backend.com/auth/google/callback
FRONTEND_URL=https://your-frontend.com
CORS_ORIGINS=https://your-frontend.com
```

---

## 🎨 Customization

### Change Avatar Colors

**`AgenticAvatar.jsx`:**

```javascript
<linearGradient id="headGradient">
  <stop offset="0%" style={{ stopColor: '#YOUR_COLOR', stopOpacity: 1 }} />
  <stop offset="100%" style={{ stopColor: '#YOUR_COLOR_2', stopOpacity: 1 }} />
</linearGradient>
```

### Adjust Widget Position

**`RAGChatWidget.css`:**

```css
.rag-chat-toggle {
  bottom: 24px;  /* Adjust vertical position */
  right: 24px;   /* Adjust horizontal position */
}
```

### Add New Subagents

**`backend/app/chat/subagents.py`:**

```python
class MyCustomAgent:
    async def process(self, content: str) -> str:
        # Your logic here
        return processed_content
```

---

## 📊 Chapter 1 Content Structure

The chatbot is trained on:

1. **Week 1**: Introduction to Physical AI
2. **Weeks 2-4**: ROS 2 (Nodes, Topics, Services, URDF)
3. **Weeks 5-7**: Gazebo & Unity (Simulation, Sensors)
4. **Weeks 8-10**: NVIDIA Isaac (Perception, Navigation)
5. **Weeks 11-13**: VLA Integration (Voice, Planning, Capstone)

---

## 🔧 Troubleshooting

### Backend Issues

**Database connection fails:**
- Check Neon Postgres connection string
- Ensure SSL mode is enabled: `?sslmode=require`

**Qdrant connection fails:**
- Verify cluster URL and API key
- Check firewall/network restrictions

**OAuth redirect fails:**
- Ensure redirect URI matches exactly in Google Console
- Check CORS settings

### Frontend Issues

**Widget doesn't appear:**
- Check browser console for errors
- Verify `framer-motion` is installed
- Clear cache and reload

**Authentication fails:**
- Verify backend is running
- Check Google OAuth credentials
- Ensure redirect URI is correct

**Chat not working:**
- Verify token in localStorage
- Check network tab for API errors
- Ensure backend endpoints are accessible

---

## 📝 API Documentation

### Authentication Endpoints

```
GET  /auth/google/login       - Initiate Google OAuth
GET  /auth/google/callback    - Handle OAuth callback
GET  /auth/verify             - Verify JWT token
POST /auth/logout             - Logout user
```

### Chat Endpoints

```
POST /chat/message            - Send chat message
POST /chat/personalize        - Personalize content
POST /chat/translate          - Translate to Urdu
POST /chat/explain-code       - Explain code snippet
GET  /chat/profile            - Get user profile
PUT  /chat/profile            - Update user profile
```

### Health Check

```
GET  /                        - Root endpoint
GET  /health                  - Health check
```

---

## 📚 Resources

- [FastAPI Documentation](https://fastapi.tiangolo.com)
- [Qdrant Documentation](https://qdrant.tech/documentation)
- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)
- [Framer Motion Docs](https://www.framer.com/motion)
- [Playwright Documentation](https://playwright.dev)
- [Docusaurus Guide](https://docusaurus.io)

---

## 🎯 Next Steps

1. ✅ Complete backend setup
2. ✅ Configure authentication
3. ✅ Index Chapter 1 content
4. ✅ Integrate widget into Docusaurus
5. ✅ Test all features
6. ✅ Deploy to production
7. 🔄 Monitor usage and iterate

---

## 📄 License

MIT License - See LICENSE file for details

---

## 🤝 Support

For issues or questions:
- Backend: Check logs with `--log-level debug`
- Frontend: Check browser console
- Tests: Run `npm run test:debug`

---

**Built with ❤️ for Physical AI Education**

Spec Kit Plus Compliant | MCP Context7 | UV Package Management | Agentic AI
