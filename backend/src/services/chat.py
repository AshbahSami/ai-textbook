import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
from uuid import uuid4
from pydantic import BaseModel
import psycopg2
from psycopg2.extras import RealDictCursor
from .models import ChatSession, ChatMessage
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ChatService:
    def __init__(self, db_url: str = os.getenv("NEON_DB_URL")):
        self.db_url = db_url
        self._init_db()

    def _init_db(self):
        """Initialize database connection and create tables if they don't exist"""
        try:
            conn = psycopg2.connect(self.db_url)
            cursor = conn.cursor()
            
            # Create chat_sessions table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS chat_sessions (
                    session_id VARCHAR(255) PRIMARY KEY,
                    user_id VARCHAR(255),
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    is_active BOOLEAN DEFAULT TRUE,
                    metadata JSONB
                )
            """)
            
            # Create chat_messages table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS chat_messages (
                    message_id VARCHAR(255) PRIMARY KEY,
                    session_id VARCHAR(255) REFERENCES chat_sessions(session_id),
                    sender_type VARCHAR(10) CHECK (sender_type IN ('user', 'assistant')),
                    content TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    context_sources JSONB,
                    metadata JSONB
                )
            """)
            
            # Create user_metadata table
            cursor.execute("""
                CREATE TABLE IF NOT EXISTS user_metadata (
                    user_id VARCHAR(255) PRIMARY KEY,
                    session_count INTEGER DEFAULT 0,
                    first_seen TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    last_seen TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    preferences JSONB
                )
            """)
            
            conn.commit()
            cursor.close()
            conn.close()
            
            logger.info("Database initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing database: {str(e)}")
            raise

    def create_session(self, user_id: Optional[str] = None, metadata: Optional[Dict[str, Any]] = None) -> str:
        """Create a new chat session"""
        session_id = str(uuid4())
        try:
            conn = psycopg2.connect(self.db_url)
            cursor = conn.cursor()
            
            cursor.execute("""
                INSERT INTO chat_sessions (session_id, user_id, metadata)
                VALUES (%s, %s, %s)
            """, (session_id, user_id, metadata))
            
            # Update user metadata if user_id is provided
            if user_id:
                cursor.execute("""
                    INSERT INTO user_metadata (user_id, session_count, first_seen, last_seen)
                    VALUES (%s, 1, CURRENT_TIMESTAMP, CURRENT_TIMESTAMP)
                    ON CONFLICT (user_id)
                    DO UPDATE SET
                        session_count = user_metadata.session_count + 1,
                        last_seen = CURRENT_TIMESTAMP
                """, (user_id,))
            
            conn.commit()
            cursor.close()
            conn.close()
            
            logger.info(f"Created new session: {session_id}")
            return session_id
        except Exception as e:
            logger.error(f"Error creating session: {str(e)}")
            raise

    def add_message(self, session_id: str, sender_type: str, content: str, 
                   context_sources: Optional[List[str]] = None, 
                   metadata: Optional[Dict[str, Any]] = None) -> str:
        """Add a message to a chat session"""
        message_id = str(uuid4())
        try:
            conn = psycopg2.connect(self.db_url)
            cursor = conn.cursor()
            
            cursor.execute("""
                INSERT INTO chat_messages (message_id, session_id, sender_type, content, context_sources, metadata)
                VALUES (%s, %s, %s, %s, %s, %s)
            """, (message_id, session_id, sender_type, content, context_sources, metadata))
            
            # Update session's updated_at timestamp
            cursor.execute("""
                UPDATE chat_sessions
                SET updated_at = CURRENT_TIMESTAMP
                WHERE session_id = %s
            """, (session_id,))
            
            conn.commit()
            cursor.close()
            conn.close()
            
            logger.info(f"Added message {message_id} to session {session_id}")
            return message_id
        except Exception as e:
            logger.error(f"Error adding message to session {session_id}: {str(e)}")
            raise

    def get_session_history(self, session_id: str) -> List[Dict[str, Any]]:
        """Get chat history for a session"""
        try:
            conn = psycopg2.connect(self.db_url)
            cursor = conn.cursor(cursor_factory=RealDictCursor)
            
            cursor.execute("""
                SELECT message_id, session_id, sender_type, content, timestamp, context_sources, metadata
                FROM chat_messages
                WHERE session_id = %s
                ORDER BY timestamp ASC
            """, (session_id,))
            
            rows = cursor.fetchall()
            cursor.close()
            conn.close()
            
            messages = [dict(row) for row in rows]
            logger.info(f"Retrieved {len(messages)} messages for session {session_id}")
            return messages
        except Exception as e:
            logger.error(f"Error getting session history for {session_id}: {str(e)}")
            return []

    def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get session information"""
        try:
            conn = psycopg2.connect(self.db_url)
            cursor = conn.cursor(cursor_factory=RealDictCursor)
            
            cursor.execute("""
                SELECT session_id, user_id, created_at, updated_at, is_active, metadata
                FROM chat_sessions
                WHERE session_id = %s
            """, (session_id,))
            
            row = cursor.fetchone()
            cursor.close()
            conn.close()
            
            if row:
                return dict(row)
            return None
        except Exception as e:
            logger.error(f"Error getting session {session_id}: {str(e)}")
            return None

    def close_session(self, session_id: str) -> bool:
        """Close a chat session"""
        try:
            conn = psycopg2.connect(self.db_url)
            cursor = conn.cursor()
            
            cursor.execute("""
                UPDATE chat_sessions
                SET is_active = FALSE
                WHERE session_id = %s
            """, (session_id,))
            
            conn.commit()
            cursor.close()
            conn.close()
            
            logger.info(f"Closed session {session_id}")
            return True
        except Exception as e:
            logger.error(f"Error closing session {session_id}: {str(e)}")
            return False