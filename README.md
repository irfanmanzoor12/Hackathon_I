# Physical AI & Humanoid Robotics - Interactive Textbook

A complete interactive textbook platform for learning Physical AI and Humanoid Robotics with an AI-powered tutor chatbot.

## Features

### Interactive Textbook (Docusaurus)
- 4 comprehensive modules covering 13 weeks of content
- Module 1: Robotic Nervous System (ROS 2)
- Module 2: Digital Twin (Gazebo & Unity)
- Module 3: AI-Robot Brain (NVIDIA Isaac)
- Module 4: Vision-Language-Action (VLA)

### AI Tutor Chatbot
- RAG-powered responses using GPT-4o
- Text selection for contextual questions
- Personalize content based on your IT background
- Translate content to Urdu
- Animated avatar with eye/mouth movements

### Authentication
- Email/Password signup and login
- Google OAuth support
- JWT token-based sessions

## Quick Start

### Prerequisites
- Node.js 18+
- Python 3.12+
- UV package manager (`pip install uv`)

### 1. Start Backend (Port 8000)
```bash
cd backend
uv venv && source .venv/bin/activate  # or .venv\Scripts\activate on Windows
uv pip install -r requirements.txt
cp .env.example .env  # Edit with your API keys
uv run uvicorn app.main:app --reload --port 8000
```

### 2. Start Auth Service (Port 3001)
```bash
cd auth-service
npm install
cp .env.example .env  # Edit with your credentials
npm start
```

### 3. Start Frontend (Port 3000)
```bash
cd frontend
npm install
npm start
```

### 4. Access the App
Open http://localhost:3000

## Environment Variables

### Backend (.env)
```env
SECRET_KEY=your-secret-key-32-chars
OPENAI_API_KEY=sk-your-openai-key
DATABASE_URL=postgresql://...  # Neon Postgres (optional)
QDRANT_URL=https://...  # Qdrant Cloud (optional)
QDRANT_API_KEY=your-qdrant-key
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-secret
```

### Auth Service (.env)
```env
AUTH_PORT=3001
AUTH_SECRET=your-auth-secret
DATABASE_URL=postgresql://...
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-secret
```

## Project Structure

```
├── frontend/           # Docusaurus textbook site
│   ├── docs/          # Course content (MDX)
│   └── src/
│       ├── components/
│       │   ├── RAGChatWidget/    # AI chatbot
│       │   ├── AuthModal/        # Login/Signup
│       │   └── ChapterActions/   # Personalize/Translate
│       └── api/                  # API clients
│
├── backend/            # FastAPI server
│   └── app/
│       ├── auth/      # Authentication
│       ├── chat/      # RAG engine & subagents
│       └── database/  # Postgres & Qdrant
│
└── auth-service/       # better-auth service
    └── src/
```

## API Endpoints

### Authentication
- `POST /auth/register` - Register new user
- `POST /auth/login` - Login with email/password
- `GET /auth/google/login` - Google OAuth

### Chat
- `POST /chat/message` - Send message to AI tutor
- `POST /chat/translate` - Translate to Urdu
- `POST /chat/personalize` - Personalize content
- `GET /chat/profile` - Get user profile

## Tech Stack

- **Frontend**: React, Docusaurus, Framer Motion
- **Backend**: FastAPI, OpenAI GPT-4o, SQLite/Postgres
- **Auth**: better-auth, JWT
- **Vector Store**: Qdrant (optional)

## Course Content

| Module | Topic | Weeks |
|--------|-------|-------|
| 1 | ROS 2 Fundamentals | 1-3 |
| 2 | Digital Twin (Gazebo & Unity) | 4-7 |
| 3 | NVIDIA Isaac | 8-10 |
| 4 | Vision-Language-Action | 11-13 |

## License

MIT License

---

Built for Physical AI & Humanoid Robotics Education
