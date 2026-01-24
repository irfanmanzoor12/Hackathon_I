# Vercel Deployment Guide

Complete guide to deploy your Physical AI RAG Chatbot to Vercel.

---

## Prerequisites

1. **Vercel Account**: Sign up at https://vercel.com
2. **GitHub Repository**: Code pushed to GitHub ✅
3. **Vercel CLI** (optional): `npm install -g vercel`

---

## Deployment Steps

### **Option 1: Deploy via Vercel Dashboard (Recommended)**

#### 1. Import Project
1. Go to https://vercel.com/new
2. Click "Import Project"
3. Connect your GitHub account
4. Select repository: `irfanmanzoor12/Physical_AI_Humanoid_Robotics_01`

#### 2. Configure Project
- **Framework Preset**: Other
- **Root Directory**: `./` (leave default)
- **Build Command**: `cd frontend && npm run build`
- **Output Directory**: `frontend/build`

#### 3. Add Environment Variables

Click "Environment Variables" and add these:

##### **Required Variables:**
```
SECRET_KEY=your-super-secret-key-min-32-chars-change-this
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=1440

GOOGLE_CLIENT_ID=your-google-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret
GOOGLE_REDIRECT_URI=https://YOUR-VERCEL-URL.vercel.app/auth/google/callback

DATABASE_URL=your-neon-postgres-connection-string

QDRANT_URL=your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=chapter_1_physical_ai

OPENAI_API_KEY=your-openai-api-key

EMBEDDING_PROVIDER=openai
CORS_ORIGINS=https://YOUR-VERCEL-URL.vercel.app
```

**💡 Use your actual values from `backend/.env` file**

⚠️ **IMPORTANT**: Replace `YOUR-VERCEL-URL` with your actual Vercel deployment URL after first deploy!

#### 4. Deploy
Click **"Deploy"** button and wait 2-3 minutes.

---

### **Option 2: Deploy via Vercel CLI**

```bash
# Install Vercel CLI
npm install -g vercel

# Login
vercel login

# Deploy
vercel

# Follow prompts:
# - Link to existing project? No
# - Project name? Physical_AI_Humanoid_Robotics_01
# - Directory? ./
# - Override settings? No

# Add environment variables
vercel env add SECRET_KEY
vercel env add OPENAI_API_KEY
# ... (add all variables from above)

# Deploy to production
vercel --prod
```

---

## Post-Deployment Configuration

### 1. Update Google OAuth Redirect URI
1. Go to Google Cloud Console: https://console.cloud.google.com/apis/credentials
2. Find your OAuth Client ID
3. Add Authorized Redirect URI:
   ```
   https://YOUR-VERCEL-URL.vercel.app/auth/google/callback
   ```

### 2. Update CORS Origins
1. Go to Vercel Dashboard → Your Project → Settings → Environment Variables
2. Update `CORS_ORIGINS`:
   ```
   https://YOUR-VERCEL-URL.vercel.app
   ```
3. Redeploy: Vercel Dashboard → Deployments → Latest → Redeploy

### 3. Update Frontend API URL
The frontend needs to know the backend URL. Create `frontend/.env.production`:
```
REACT_APP_API_URL=https://YOUR-VERCEL-URL.vercel.app
```

---

## Vercel Deployment Structure

```
Physical_AI_Humanoid_Robotics_01/
├── vercel.json              ← Main Vercel config
├── .vercelignore           ← Files to ignore
├── backend/
│   ├── vercel.json         ← Backend config
│   └── app/
│       └── main.py         ← FastAPI (Serverless Function)
└── frontend/
    ├── build/              ← Static files (auto-generated)
    └── package.json        ← Build config
```

---

## API Endpoints After Deployment

- **Frontend**: `https://YOUR-VERCEL-URL.vercel.app`
- **Backend API**: `https://YOUR-VERCEL-URL.vercel.app/api/`
- **Health Check**: `https://YOUR-VERCEL-URL.vercel.app/health`
- **Docs**: `https://YOUR-VERCEL-URL.vercel.app/docs`

---

## Troubleshooting

### Build Fails
**Error**: `Module not found`
**Solution**: Ensure `requirements.txt` includes all dependencies

### API Routes Don't Work
**Error**: 404 on `/api/*`
**Solution**: Check `vercel.json` routes configuration

### CORS Errors
**Error**: `Access-Control-Allow-Origin`
**Solution**:
1. Update `CORS_ORIGINS` environment variable
2. Add your Vercel URL
3. Redeploy

### OAuth Fails
**Error**: `redirect_uri_mismatch`
**Solution**:
1. Update Google OAuth redirect URI with Vercel URL
2. Update `GOOGLE_REDIRECT_URI` env variable

### Serverless Function Timeout
**Error**: Function timeout (10s limit on free tier)
**Solution**:
- Upgrade Vercel plan for 60s timeout
- Or use Railway/Render for backend (no timeout limits)

---

## Alternative: Split Deployment

If Vercel serverless has limitations, deploy separately:

### Frontend on Vercel
1. Deploy only `frontend/` directory
2. Set `REACT_APP_API_URL` to your backend URL

### Backend on Railway/Render
```bash
# Railway
railway init
railway up

# Render
# Connect GitHub repo
# Set root directory: backend/
# Build command: pip install -r requirements.txt
# Start command: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

---

## Cost Estimation

### Vercel (Free Tier)
- 100GB Bandwidth
- Unlimited Serverless Functions
- ✅ **Cost: $0**

### If Exceeded (Pro Tier: $20/month)
- 1TB Bandwidth
- 1000 serverless hours
- 60s function timeout

---

## Next Steps After Deployment

1. ✅ Test all features
2. ✅ Update GitHub README with live URL
3. ✅ Monitor with Vercel Analytics
4. ✅ Set up custom domain (optional)

---

## Live URLs

Once deployed, you'll have:
- **App**: https://YOUR-PROJECT.vercel.app
- **API**: https://YOUR-PROJECT.vercel.app/api
- **Docs**: https://YOUR-PROJECT.vercel.app/docs

---

## Support

- Vercel Docs: https://vercel.com/docs
- Vercel Discord: https://vercel.com/discord
- GitHub Issues: https://github.com/irfanmanzoor12/Physical_AI_Humanoid_Robotics_01/issues
