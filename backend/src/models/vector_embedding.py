from datetime import datetime
from typing import List, Optional, Dict, Any
from pydantic import BaseModel


class VectorEmbedding(BaseModel):
    """
    Mathematical representations of documentation chunks stored in Qdrant for similarity search
    """
    id: str
    document_id: str
    content_chunk: str
    embedding_vector: List[float]
    chunk_metadata: Optional[Dict[str, Any]] = None
    created_at: datetime