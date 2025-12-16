"""
Request models for the contextual RAG lookup service
"""
from pydantic import BaseModel, Field
from typing import Optional
from uuid import UUID


class ContextQueryRequest(BaseModel):
    """
    Request model for the contextual query endpoint
    """
    session_id: str  # Session identifier (UUID string)
    selected_text: str  # The exact text string the user highlighted
    full_page_url: str  # The URL of the page where the selection occurred
    surrounding_context: str  # The text surrounding the selected text


class ContextQueryResponse(BaseModel):
    """
    Response model for the contextual query endpoint
    """
    explanation: str  # The generated explanation for the selected text
    source_url: str  # The original URL from the request
    session_id: str  # The session identifier returned to client
    supplemental_context_used: Optional[bool] = False  # Whether Qdrant supplemental context was used


class ContextualQuery(BaseModel):
    """
    Internal model for processing contextual queries
    """
    session_id: str
    selected_text: str
    full_page_url: str
    primary_context: str  # The surrounding_context from the request
    supplemental_context: Optional[list[str]] = []  # Retrieved from Qdrant if applicable
    query_embedding: Optional[list[float]] = None  # Embedding of the selected_text if Qdrant lookup occurs


class QueryResult(BaseModel):
    """
    Result model for a contextual query processing
    """
    explanation: str
    supplemental_context_used: bool
    context_quality_score: Optional[float] = None  # Confidence or relevance score
    processing_time_ms: int