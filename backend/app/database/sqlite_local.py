"""
Local SQLite Database - Fallback for offline/development
"""

import aiosqlite
import os
from typing import Optional, Dict, Any
from datetime import datetime
import hashlib
import secrets
import logging

logger = logging.getLogger(__name__)

# Database path
DB_PATH = "local_humanoid.db"
db: Optional[aiosqlite.Connection] = None


async def init_local_db():
    """Initialize local SQLite database"""
    global db
    try:
        db = await aiosqlite.connect(DB_PATH)
        db.row_factory = aiosqlite.Row
        
        # Create tables
        await db.execute("""
            CREATE TABLE IF NOT EXISTS users (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                email TEXT UNIQUE NOT NULL,
                password_hash TEXT,
                name TEXT NOT NULL,
                picture TEXT,
                software_background TEXT,
                hardware_background TEXT,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                last_login TEXT,
                updated_at TEXT DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        await db.execute("""
            CREATE TABLE IF NOT EXISTS conversation_history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                user_id INTEGER,
                session_id TEXT NOT NULL,
                role TEXT NOT NULL,
                content TEXT NOT NULL,
                context_metadata TEXT,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY (user_id) REFERENCES users(id)
            )
        """)
        
        await db.commit()
        logger.info("Local SQLite database initialized")
        return True
        
    except Exception as e:
        logger.error(f"Failed to initialize local database: {str(e)}")
        return False


async def close_local_db():
    """Close local database"""
    if db:
        await db.close()
        logger.info("Local database closed")


def hash_password(password: str) -> str:
    """Hash password using SHA-256 with salt"""
    salt = secrets.token_hex(32)
    pwd_hash = hashlib.sha256((password + salt).encode('utf-8')).hexdigest()
    return f"{salt}${pwd_hash}"


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify password"""
    try:
        salt, pwd_hash = hashed_password.split('$')
        test_hash = hashlib.sha256((plain_password + salt).encode('utf-8')).hexdigest()
        return test_hash == pwd_hash
    except:
        return False


async def get_user_by_email(email: str) -> Optional[Dict[str, Any]]:
    """Get user by email"""
    if not db:
        return None
        
    async with db.execute("SELECT * FROM users WHERE email = ?", (email,)) as cursor:
        row = await cursor.fetchone()
        return dict(row) if row else None


async def get_user_by_id(user_id: int) -> Optional[Dict[str, Any]]:
    """Get user by ID"""
    if not db:
        return None
        
    async with db.execute("SELECT * FROM users WHERE id = ?", (user_id,)) as cursor:
        row = await cursor.fetchone()
        return dict(row) if row else None


async def create_user(email: str, name: str, password: Optional[str] = None, **kwargs) -> Dict[str, Any]:
    """Create new user"""
    if not db:
        raise Exception("Database not initialized")
    
    password_hash = hash_password(password) if password else None
    
    cursor = await db.execute("""
        INSERT INTO users (email, name, password_hash, picture, software_background, hardware_background, last_login)
        VALUES (?, ?, ?, ?, ?, ?, ?)
    """, (
        email, 
        name, 
        password_hash,
        kwargs.get('picture'),
        kwargs.get('software_background'),
        kwargs.get('hardware_background'),
        datetime.utcnow().isoformat()
    ))
    
    await db.commit()
    
    user_id = cursor.lastrowid
    return await get_user_by_id(user_id)


async def update_user(user_id: int, update_data: Dict[str, Any]) -> Dict[str, Any]:
    """Update user profile"""
    if not db:
        raise Exception("Database not initialized")
    
    # Build update query
    fields = []
    values = []
    
    for key, value in update_data.items():
        if value is not None and key != 'id':
            fields.append(f"{key} = ?")
            values.append(value)
    
    if not fields:
        return await get_user_by_id(user_id)
    
    fields.append("updated_at = ?")
    values.append(datetime.utcnow().isoformat())
    values.append(user_id)
    
    query = f"UPDATE users SET {', '.join(fields)} WHERE id = ?"
    await db.execute(query, values)
    await db.commit()
    
    return await get_user_by_id(user_id)


async def save_conversation(user_id: int, session_id: str, role: str, content: str, metadata: Optional[Dict] = None):
    """Save conversation message"""
    if not db:
        return
        
    import json
    await db.execute("""
        INSERT INTO conversation_history (user_id, session_id, role, content, context_metadata)
        VALUES (?, ?, ?, ?, ?)
    """, (user_id, session_id, role, content, json.dumps(metadata) if metadata else None))
    await db.commit()


async def get_conversation_history(session_id: str, limit: int = 10) -> list:
    """Get conversation history"""
    if not db:
        return []
        
    async with db.execute("""
        SELECT role, content, context_metadata, created_at
        FROM conversation_history
        WHERE session_id = ?
        ORDER BY created_at DESC
        LIMIT ?
    """, (session_id, limit)) as cursor:
        rows = await cursor.fetchall()
        return [dict(row) for row in reversed(rows)]
