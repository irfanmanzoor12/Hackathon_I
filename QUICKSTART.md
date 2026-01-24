# Quick Start Guide
## Get Physical AI RAG Chatbot Running in 15 Minutes

This guide gets you up and running with the complete RAG chatbot system.

---

## Prerequisites Checklist

- [ ] Python 3.12+ installed
- [ ] Node.js 18+ installed
- [ ] Git installed
- [ ] Google account (for OAuth)
- [ ] OpenAI API key (https://platform.openai.com/api-keys)

---

## Step 1: Clone Repository (1 min)

```bash
cd D:\Irfan\humanoid
# Already done if you're reading this!
```

---

## Step 2: Get API Keys (5 min)

### OpenAI API Key

1. Go to https://platform.openai.com/api-keys
2. Click "Create new secret key"
3. Copy the key (starts with `sk-`)

### Neon Postgres (Free Tier)

1. Go to https://neon.tech
2. Sign up / Log in
3. Create new project: `humanoid-rag`
4. Copy connection string from dashboard

### Qdrant Cloud (Free Tier)

1. Go to https://cloud.qdrant.io
2. Sign up / Log in
3. Create cluster (Free tier: 1GB)
4. Copy cluster URL and API key

### Google OAuth

1. Go to https://console.cloud.google.com/apis/credentials
2. Create project: "Physical AI RAG"
3. Create OAuth 2.0 Client ID (Web application)
4. Add redirect URI: `http://localhost:8000/auth/google/callback`
5. Copy Client ID and Client Secret

---

## Step 3: Backend Setup (3 min)

```bash
# Navigate to backend
cd backend

# Install UV
pip install uv

# Create virtual environment
uv venv

# Activate (Windows)
.venv\Scripts\activate

# Activate (Mac/Linux)
# source .venv/bin/activate

# Install dependencies
uv pip install -r requirements.txt

# Create .env from template
copy .env.example .env
# Mac/Linux: cp .env.example .env
```

**Edit `.env` with your API keys:**

```env
SECRET_KEY=your-32-char-secret-key-change-this
GOOGLE_CLIENT_ID=your-google-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret
DATABASE_URL=postgresql://user:pass@host.neon.tech/humanoid_rag?sslmode=require
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
OPENAI_API_KEY=sk-your-openai-key
```

---

## Step 4: Index Chapter 1 Content (2 min)

```bash
# Still in backend directory
python scripts/index_chapter1.py
```

You should see:
```
✅ Chapter 1 Content Indexed Successfully!
Total sections indexed: 23
```

---

## Step 5: Start Backend (1 min)

```bash
uvicorn app.main:app --reload
```

You should see:
```
INFO:     Uvicorn running on http://127.0.0.1:8000
✅ Database and Vector Store initialized
```

**Test it:** Open http://localhost:8000/docs

---

## Step 6: Frontend Setup (3 min)

Open a **new terminal**:

```bash
# Navigate to frontend (or your Docusaurus site)
cd D:\Irfan\humanoid\frontend

# Install dependencies
npm install framer-motion

# Create .env
echo REACT_APP_API_URL=http://localhost:8000 > .env
```

**If using existing Docusaurus site:**

1. Copy `src/components/RAGChatWidget/` to your site
2. Copy `src/api/chatAPI.js` to your site
3. Create `src/theme/Root.js` (see DOCUSAURUS_INTEGRATION.md)
4. Create `src/pages/auth/callback.js` (see DOCUSAURUS_INTEGRATION.md)

---

## Step 7: Start Frontend

```bash
# In your Docusaurus site
npm start
```

Opens: http://localhost:3000

---

## Step 8: Test the System! (Fun part!)

### Test 1: Widget Appears

✅ You should see a blue floating button in bottom-right corner with animated avatar

### Test 2: Authentication

1. Click the floating button
2. Click "Sign in with Google"
3. Complete Google login
4. You should be redirected back with chatbot open

### Test 3: Chat

1. Type: **"What is ROS 2?"**
2. Press Enter
3. Watch the avatar animate (thinking → responding)
4. See the response with sources!

### Test 4: Text Selection

1. Add some text to your page (or use Chapter 1 content)
2. Select 2-3 sentences
3. Widget should auto-open with selected text preview
4. Type: **"Explain this"**
5. Get contextual answer!

### Test 5: Personalization

1. Type: **"Explain SLAM"**
2. Click "Personalize" button
3. (First time: it will ask for your IT background)
4. Get personalized response!

### Test 6: Translation

1. Type any question
2. Click "Translate" button
3. Get response in Urdu!

---

## Common Issues & Fixes

### Backend won't start

**Error: Database connection failed**
- Check Neon connection string is correct
- Ensure `?sslmode=require` is at the end

**Error: Qdrant connection failed**
- Verify cluster URL and API key
- Try accessing URL in browser (should show Qdrant dashboard)

**Error: Module not found**
- Ensure virtual environment is activated
- Run `uv pip install -r requirements.txt` again

### Frontend issues

**Widget doesn't appear**
- Check browser console (F12)
- Ensure `framer-motion` is installed
- Try clearing cache (Ctrl+Shift+R)

**Authentication fails**
- Verify backend is running (http://localhost:8000/health)
- Check Google redirect URI matches exactly
- Check browser console for errors

**Chat doesn't work**
- Open Network tab in DevTools
- Check for 401/403 errors (auth issue)
- Check for 500 errors (backend issue)
- Verify token is in localStorage: `localStorage.getItem('rag_token')`

---

## Next Steps

### Add Your Own Content

Edit `backend/scripts/index_chapter1.py` and add your sections:

```python
{
    "content": "Your content here...",
    "metadata": {
        "section": "Your Section Name",
        "week": "1",
        "topic": "Your Topic"
    }
}
```

Run: `python scripts/index_chapter1.py`

### Customize Avatar

Edit colors in `AgenticAvatar.jsx`:

```javascript
<stop offset="0%" style={{ stopColor: '#YOUR_COLOR' }} />
```

### Set Your IT Background

After logging in:
1. Click user name at bottom
2. Update profile
3. Future responses will be personalized!

---

## Production Deployment

### Quick Deploy to Railway

```bash
# Install Railway CLI
npm install -g railway

# Login
railway login

# Deploy backend
cd backend
railway init
railway up

# Get backend URL
railway domain

# Update frontend .env with production URL
REACT_APP_API_URL=https://your-backend.railway.app

# Deploy frontend to Vercel
cd your-docusaurus-site
vercel deploy
```

### Update Google OAuth

Add production redirect URI in Google Console:
```
https://your-backend.railway.app/auth/google/callback
```

---

## Success! 🎉

You now have a fully functional RAG chatbot with:
- ✅ Google OAuth authentication
- ✅ Agentic animated avatar
- ✅ Context-aware responses
- ✅ Text selection support
- ✅ Personalization
- ✅ Translation
- ✅ MCP Context7 multi-turn memory

**Ready for Chapter 1 testing!**

---

## Get Help

- Check `README.md` for detailed documentation
- Check `DOCUSAURUS_INTEGRATION.md` for integration guide
- Run backend with debug: `uvicorn app.main:app --log-level debug`
- Check browser console (F12) for frontend errors

---

**Time to teach Physical AI! 🤖**
