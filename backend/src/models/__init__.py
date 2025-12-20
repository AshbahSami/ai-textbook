from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

class DocumentationContent(BaseModel):
    id: str
    title: str
    url: str
    content: str
    metadata: Optional[Dict[str, Any]] = {}
    created_at: datetime
    updated_at: datetime
    source_hash: str

class VectorEmbedding(BaseModel):
    id: str
    document_id: str
    content_chunk: str
    embedding_vector: list  # Array of floats
    chunk_metadata: Optional[Dict[str, Any]] = {}
    created_at: datetime

class ChatSession(BaseModel):
    session_id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    is_active: bool = True
    metadata: Optional[Dict[str, Any]] = {}

class ChatMessage(BaseModel):
    message_id: str
    session_id: str
    sender_type: str  # 'user' or 'assistant'
    content: str
    timestamp: datetime
    context_sources: Optional[list] = []
    metadata: Optional[Dict[str, Any]] = {}

class UserMetadata(BaseModel):
    user_id: str
    session_count: int = 0
    first_seen: datetime
    last_seen: datetime
    preferences: Optional[Dict[str, Any]] = {}