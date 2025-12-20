from datetime import datetime
from typing import List, Optional, Dict, Any
from pydantic import BaseModel


class ChatMessage(BaseModel):
    """
    Represents a single message in a chat conversation
    """
    message_id: str
    session_id: str
    sender_type: str  # 'user' or 'assistant'
    content: str
    timestamp: datetime
    context_sources: Optional[List[str]] = None
    metadata: Optional[Dict[str, Any]] = None