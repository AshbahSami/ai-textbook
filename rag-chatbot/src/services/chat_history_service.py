"""
Chat history service for Neon DB interaction
"""
import logging
from typing import List, Optional
from sqlalchemy.orm import Session
from sqlalchemy import text
from datetime import datetime, timedelta
from fastapi import HTTPException, status
from ..models.request_models import ContextQueryRequest
from ..config.database import SessionLocal, engine
from ..utils.exceptions import DatabaseError
import uuid


# Create table if it doesn't exist
def create_chat_interactions_table():
    with engine.connect() as conn:
        # Create the chat_interactions table if it doesn't exist
        conn.execute(text("""
            CREATE TABLE IF NOT EXISTS chat_interactions (
                id SERIAL PRIMARY KEY,
                session_id TEXT NOT NULL,
                user_query TEXT NOT NULL,
                context_used TEXT NOT NULL,
                llm_response TEXT NOT NULL,
                source_url TEXT NOT NULL,
                timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW()
            )
        """))
        # Create indexes if they don't exist
        conn.execute(text("""
            CREATE INDEX IF NOT EXISTS idx_chat_interactions_session_id
            ON chat_interactions(session_id)
        """))
        conn.execute(text("""
            CREATE INDEX IF NOT EXISTS idx_chat_interactions_timestamp
            ON chat_interactions(timestamp)
        """))
        conn.commit()


class ChatHistoryService:
    """
    Service for handling chat history operations in Neon DB
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        # Ensure the table exists
        create_chat_interactions_table()

    def save_interaction(
        self,
        session_id: str,
        user_query: str,
        context_used: str,
        llm_response: str,
        source_url: str
    ) -> int:
        """
        Save a chat interaction to the database
        Returns the ID of the saved interaction
        """
        try:
            with SessionLocal() as db:
                query = text("""
                    INSERT INTO chat_interactions
                    (session_id, user_query, context_used, llm_response, source_url)
                    VALUES (:session_id, :user_query, :context_used, :llm_response, :source_url)
                    RETURNING id
                """)

                result = db.execute(
                    query,
                    {
                        "session_id": session_id,
                        "user_query": user_query,
                        "context_used": context_used,
                        "llm_response": llm_response,
                        "source_url": source_url
                    }
                )

                interaction_id = result.fetchone()[0]
                db.commit()

                self.logger.info(f"Saved interaction with ID: {interaction_id}")
                return interaction_id
        except Exception as e:
            self.logger.error(f"Error saving interaction: {str(e)}")
            raise DatabaseError(f"Failed to save chat interaction: {str(e)}")

    def get_session_history(self, session_id: str) -> List[dict]:
        """
        Retrieve chat history for a specific session
        """
        try:
            with SessionLocal() as db:
                query = text("""
                    SELECT id, user_query, llm_response, source_url, timestamp
                    FROM chat_interactions
                    WHERE session_id = :session_id
                    ORDER BY timestamp ASC
                """)

                result = db.execute(query, {"session_id": session_id})
                rows = result.fetchall()

                history = []
                for row in rows:
                    history.append({
                        "id": row[0],
                        "user_query": row[1],
                        "llm_response": row[2],
                        "source_url": row[3],
                        "timestamp": row[4]
                    })

                self.logger.info(f"Retrieved {len(history)} interactions for session: {session_id}")
                return history
        except Exception as e:
            self.logger.error(f"Error retrieving session history: {str(e)}")
            raise DatabaseError(f"Failed to retrieve chat history: {str(e)}")

    def get_all_sessions(self) -> List[str]:
        """
        Get all unique session IDs
        """
        try:
            with SessionLocal() as db:
                query = text("""
                    SELECT DISTINCT session_id
                    FROM chat_interactions
                    ORDER BY MAX(timestamp) DESC
                """)

                result = db.execute(query)
                rows = result.fetchall()

                sessions = [row[0] for row in rows]
                self.logger.info(f"Retrieved {len(sessions)} unique sessions")
                return sessions
        except Exception as e:
            self.logger.error(f"Error retrieving sessions: {str(e)}")
            raise DatabaseError(f"Failed to retrieve sessions: {str(e)}")

    def validate_session(self, session_id: str) -> bool:
        """
        Validate if a session exists and is active
        """
        try:
            with SessionLocal() as db:
                query = text("""
                    SELECT COUNT(*)
                    FROM chat_interactions
                    WHERE session_id = :session_id
                """)

                result = db.execute(query, {"session_id": session_id})
                count = result.fetchone()[0]

                is_valid = count > 0
                self.logger.info(f"Session {session_id} validation: {'valid' if is_valid else 'invalid'}")
                return is_valid
        except Exception as e:
            self.logger.error(f"Error validating session: {str(e)}")
            raise DatabaseError(f"Failed to validate session: {str(e)}")

    def cleanup_old_sessions(self, days_to_keep: int = 30) -> int:
        """
        Clean up sessions older than the specified number of days
        Returns the number of deleted records
        """
        try:
            cutoff_date = datetime.now() - timedelta(days=days_to_keep)

            with SessionLocal() as db:
                query = text("""
                    DELETE FROM chat_interactions
                    WHERE timestamp < :cutoff_date
                """)

                result = db.execute(query, {"cutoff_date": cutoff_date})
                deleted_count = result.rowcount
                db.commit()

                self.logger.info(f"Cleaned up {deleted_count} old interactions")
                return deleted_count
        except Exception as e:
            self.logger.error(f"Error cleaning up old sessions: {str(e)}")
            raise DatabaseError(f"Failed to clean up old sessions: {str(e)}")

    def get_session_summary(self, session_id: str) -> dict:
        """
        Get a summary of a session including count of interactions, first and last interaction times
        """
        try:
            with SessionLocal() as db:
                query = text("""
                    SELECT
                        COUNT(*) as interaction_count,
                        MIN(timestamp) as first_interaction,
                        MAX(timestamp) as last_interaction
                    FROM chat_interactions
                    WHERE session_id = :session_id
                """)

                result = db.execute(query, {"session_id": session_id})
                row = result.fetchone()

                if row[0] == 0:  # No interactions for this session
                    return {
                        "session_id": session_id,
                        "interaction_count": 0,
                        "first_interaction": None,
                        "last_interaction": None
                    }

                summary = {
                    "session_id": session_id,
                    "interaction_count": row[0],
                    "first_interaction": row[1],
                    "last_interaction": row[2]
                }

                self.logger.info(f"Retrieved summary for session: {session_id}")
                return summary
        except Exception as e:
            self.logger.error(f"Error getting session summary: {str(e)}")
            raise DatabaseError(f"Failed to get session summary: {str(e)}")