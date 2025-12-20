from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel


class ChatSession(BaseModel):
    """
    Represents a user's conversation history with timestamps and metadata
    """
    session_id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    is_active: bool = True
    metadata: Optional[Dict[str, Any]] = None