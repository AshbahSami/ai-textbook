import psycopg2
from psycopg2.extras import RealDictCursor
import os
from typing import Optional


class DatabaseConnection:
    """
    Utility class for managing database connections to Neon DB (PostgreSQL)
    """
    
    def __init__(self, connection_url: Optional[str] = None):
        self.connection_url = connection_url or os.getenv("NEON_DB_URL")
        if not self.connection_url:
            raise ValueError("Neon DB URL not provided and NEON_DB_URL environment variable not set")
    
    def get_connection(self):
        """
        Get a new database connection
        """
        return psycopg2.connect(self.connection_url)
    
    def get_cursor(self, cursor_factory=RealDictCursor):
        """
        Get a new database connection with a cursor
        """
        conn = self.get_connection()
        cursor = conn.cursor(cursor_factory=cursor_factory)
        return conn, cursor
    
    def execute_query(self, query: str, params: tuple = None):
        """
        Execute a query and return the results
        """
        conn, cursor = self.get_cursor()
        
        try:
            cursor.execute(query, params)
            result = cursor.fetchall()
            conn.commit()
            return result
        except Exception as e:
            conn.rollback()
            raise e
        finally:
            cursor.close()
            conn.close()
    
    def execute_update(self, query: str, params: tuple = None):
        """
        Execute an update/insert/delete query
        """
        conn, cursor = self.get_cursor()
        
        try:
            cursor.execute(query, params)
            conn.commit()
            return cursor.rowcount
        except Exception as e:
            conn.rollback()
            raise e
        finally:
            cursor.close()
            conn.close()


# Global instance for easy access
db_connection = DatabaseConnection()