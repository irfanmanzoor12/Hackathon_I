"""
PostgreSQL Database - Neon Serverless
User profiles and personalization data
"""

import asyncpg
from typing import Optional, Dict, Any
from datetime import datetime
import logging

from app.config import settings
from app.auth.schemas import UserCreate

logger = logging.getLogger(__name__)

# Connection pool
pool: Optional[asyncpg.Pool] = None


async def init_db():
    """Initialize database connection pool"""
    global pool
    try:
        pool = await asyncpg.create_pool(
            settings.DATABASE_URL,
            min_size=5,
            max_size=20
        )
        logger.info("✅ PostgreSQL connection pool created")

        # Create tables
        await create_tables()

    except Exception as e:
        logger.error(f"❌ Failed to initialize database: {str(e)}")
        raise


async def close_db():
    """Close database connection pool"""
    global pool
    if pool:
        await pool.close()
        logger.info("✅ PostgreSQL connection pool closed")


async def create_tables():
    """Create database tables"""
    async with pool.acquire() as conn:
        await conn.execute("""
            CREATE TABLE IF NOT EXISTS users (
                id SERIAL PRIMARY KEY,
                email VARCHAR(255) UNIQUE NOT NULL,
                name VARCHAR(255) NOT NULL,
                picture TEXT,
                software_background TEXT,
                hardware_background TEXT,
                created_at TIMESTAMP DEFAULT NOW(),
                last_login TIMESTAMP,
                updated_at TIMESTAMP DEFAULT NOW()
            );

            CREATE TABLE IF NOT EXISTS conversation_history (
                id SERIAL PRIMARY KEY,
                user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
                session_id VARCHAR(255) NOT NULL,
                role VARCHAR(50) NOT NULL,
                content TEXT NOT NULL,
                context_metadata JSONB,
                created_at TIMESTAMP DEFAULT NOW()
            );

            CREATE INDEX IF NOT EXISTS idx_user_email ON users(email);
            CREATE INDEX IF NOT EXISTS idx_conversation_session ON conversation_history(session_id);
            CREATE INDEX IF NOT EXISTS idx_conversation_user ON conversation_history(user_id);
        """)
        logger.info("✅ Database tables created/verified")


async def get_user_by_email(email: str) -> Optional[Dict[str, Any]]:
    """Get user by email"""
    async with pool.acquire() as conn:
        row = await conn.fetchrow(
            "SELECT * FROM users WHERE email = $1",
            email
        )
        return dict(row) if row else None


async def get_user_by_id(user_id: int) -> Optional[Dict[str, Any]]:
    """Get user by ID"""
    async with pool.acquire() as conn:
        row = await conn.fetchrow(
            "SELECT * FROM users WHERE id = $1",
            user_id
        )
        return dict(row) if row else None


async def create_user(user_data: UserCreate) -> Dict[str, Any]:
    """Create new user"""
    async with pool.acquire() as conn:
        row = await conn.fetchrow("""
            INSERT INTO users (email, name, picture, software_background, hardware_background, last_login)
            VALUES ($1, $2, $3, $4, $5, $6)
            RETURNING *
        """, user_data.email, user_data.name, user_data.picture,
            user_data.software_background, user_data.hardware_background, datetime.utcnow())
        return dict(row)


async def update_user(user_id: int, update_data: Dict[str, Any]) -> Dict[str, Any]:
    """Update user profile"""
    fields = []
    values = []
    idx = 1

    for key, value in update_data.items():
        if value is not None:
            fields.append(f"{key} = ${idx}")
            values.append(value)
            idx += 1

    if not fields:
        return await get_user_by_id(user_id)

    fields.append(f"updated_at = ${idx}")
    values.append(datetime.utcnow())
    values.append(user_id)

    query = f"UPDATE users SET {', '.join(fields)} WHERE id = ${idx + 1} RETURNING *"

    async with pool.acquire() as conn:
        row = await conn.fetchrow(query, *values)
        return dict(row) if row else None


async def save_conversation(user_id: int, session_id: str, role: str, content: str, metadata: Optional[Dict] = None):
    """Save conversation message"""
    async with pool.acquire() as conn:
        await conn.execute("""
            INSERT INTO conversation_history (user_id, session_id, role, content, context_metadata)
            VALUES ($1, $2, $3, $4, $5)
        """, user_id, session_id, role, content, metadata)


async def get_conversation_history(session_id: str, limit: int = 10) -> list:
    """Get conversation history for a session"""
    async with pool.acquire() as conn:
        rows = await conn.fetch("""
            SELECT role, content, context_metadata, created_at
            FROM conversation_history
            WHERE session_id = $1
            ORDER BY created_at DESC
            LIMIT $2
        """, session_id, limit)
        return [dict(row) for row in reversed(rows)]
