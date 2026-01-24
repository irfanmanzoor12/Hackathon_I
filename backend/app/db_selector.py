"""
Database Selector Module
Provides dynamic database module selection without circular imports
"""

# Global flags for database status
use_local_db = False
use_local_qdrant = True


def get_db_module():
    """Get the appropriate database module"""
    if use_local_db:
        from app.database import sqlite_local
        return sqlite_local
    else:
        from app.database import postgres
        return postgres
