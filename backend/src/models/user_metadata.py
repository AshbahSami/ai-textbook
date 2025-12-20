from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel


class UserMetadata(BaseModel):
    """
    Additional information about users interacting with the chatbot
    """
    user_id: str
    session_count: int = 0
    first_seen: datetime
    last_seen: datetime
    preferences: Optional[Dict[str, Any]] = None