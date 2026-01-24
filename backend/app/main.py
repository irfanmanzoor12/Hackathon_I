"""
FastAPI Main Application - RAG Chatbot with Google OAuth
Physical AI & Humanoid Robotics - Chapter 1 Testing
"""

from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from contextlib import asynccontextmanager
import logging

from app.config import settings
from app.auth.routes import router as auth_router
from app.chat.routes import router as chat_router
import app.db_selector as db_selector

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events"""
    # Startup
    logger.info("Starting Physical AI RAG Chatbot Backend")

    # Try Neon Postgres, fallback to local SQLite
    try:
        from app.database.postgres import init_db, close_db as close_postgres
        await init_db()
        logger.info("Connected to Neon Postgres")
        db_selector.use_local_db = False
        app.state.close_db = close_postgres
    except Exception as e:
        logger.warning(f"Failed to connect to Neon Postgres: {str(e)}")
        logger.info("Falling back to local SQLite database")
        from app.database.sqlite_local import init_local_db, close_local_db
        await init_local_db()
        db_selector.use_local_db = True
        app.state.close_db = close_local_db

    # Try Qdrant Cloud (optional - RAG will work without it)
    try:
        from app.database.qdrant import init_qdrant, close_qdrant
        await init_qdrant()
        logger.info("Connected to Qdrant Cloud")
        db_selector.use_local_qdrant = False
        app.state.close_qdrant = close_qdrant
    except Exception as e:
        logger.warning(f"Failed to connect to Qdrant Cloud: {str(e)}")
        logger.info("RAG retrieval will use fallback mode")
        db_selector.use_local_qdrant = True
        app.state.close_qdrant = lambda: None

    logger.info("Backend startup complete!")

    yield

    # Shutdown
    logger.info("Shutting down backend")
    if hasattr(app.state, 'close_db'):
        await app.state.close_db()
    if hasattr(app.state, 'close_qdrant'):
        await app.state.close_qdrant()
    logger.info("Connections closed")


app = FastAPI(
    title="Physical AI RAG Chatbot API",
    description="RAG-powered chatbot with Google OAuth for Chapter 1: Physical AI & Humanoid Robotics",
    version="1.0.0",
    lifespan=lifespan
)

# CORS Configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Security
security = HTTPBearer()

# Include routers
app.include_router(auth_router, prefix="/auth", tags=["Authentication"])
app.include_router(chat_router, prefix="/chat", tags=["Chat"])


@app.get("/")
async def root():
    """Root endpoint"""
    return {
        "message": "Physical AI RAG Chatbot API",
        "version": "1.0.0",
        "chapter": "Chapter 1: Physical AI & Humanoid Robotics",
        "status": "operational"
    }


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "database": "connected",
        "vector_store": "connected"
    }


# Vercel serverless handler
handler = app
