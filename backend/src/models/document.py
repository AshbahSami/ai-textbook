from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel


class DocumentationContent(BaseModel):
    """
    Represents the text and structured information from the Docusaurus site that gets indexed and retrieved
    """
    id: str
    title: str
    url: str
    content: str
    metadata: Optional[Dict[str, Any]] = None
    created_at: datetime
    updated_at: datetime
    source_hash: str