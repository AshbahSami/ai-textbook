from fastapi import APIRouter, HTTPException, UploadFile, File
from pydantic import BaseModel
from typing import List, Dict, Any
import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from ..services.ingestion import DocumentIngestionService
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize router
router = APIRouter()

# Initialize services
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_HOST", "localhost"),
    port=os.getenv("QDRANT_PORT", 6333),
    api_key=os.getenv("QDRANT_API_KEY")
)
ingestion_service = DocumentIngestionService(qdrant_client)

class Document(BaseModel):
    id: str
    title: str
    url: str
    content: str
    metadata: Dict[str, Any] = {}

class IngestRequest(BaseModel):
    documents: List[Document]

class IngestFromUrlRequest(BaseModel):
    url: str
    max_pages: int = 10

@router.post("/documents")
async def ingest_documents(request: IngestRequest):
    try:
        # Convert Pydantic models to dictionaries for ingestion service
        docs_data = [doc.dict() for doc in request.documents]
        results = ingestion_service.ingest_documents(docs_data)
        return {"results": results, "total_processed": len(results)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting documents: {str(e)}")

@router.post("/url")
async def ingest_from_url(request: IngestFromUrlRequest):
    try:
        # Fetch the main page
        response = requests.get(request.url)
        response.raise_for_status()
        
        # Parse the HTML content
        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Extract text content
        title = soup.find('title')
        title_text = title.get_text() if title else "Untitled"
        
        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()
        
        # Get text content
        text_content = soup.get_text()
        
        # Clean up text (remove extra whitespace)
        lines = (line.strip() for line in text_content.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text_content = ' '.join(chunk for chunk in chunks if chunk)
        
        # Create document
        doc = {
            "id": request.url.replace("https://", "").replace("http://", "").replace("/", "_").replace(".", "_"),
            "title": title_text,
            "url": request.url,
            "content": text_content,
            "metadata": {"source_type": "url", "ingested_at": "datetime.now().isoformat()"}
        }
        
        # Ingest the document
        result = ingestion_service.ingest_document(
            document_id=doc["id"],
            title=doc["title"],
            url=doc["url"],
            content=doc["content"],
            metadata=doc["metadata"]
        )
        
        return {"result": result, "document_id": doc["id"]}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting from URL: {str(e)}")

@router.post("/file")
async def ingest_from_file(file: UploadFile = File(...)):
    try:
        # Read file content
        content = await file.read()
        content_str = content.decode('utf-8')
        
        # Create document
        doc_id = file.filename.replace("/", "_").replace(".", "_")
        doc = {
            "id": doc_id,
            "title": file.filename,
            "url": f"file://{file.filename}",
            "content": content_str,
            "metadata": {"source_type": "file", "filename": file.filename, "ingested_at": "datetime.now().isoformat()"}
        }
        
        # Ingest the document
        result = ingestion_service.ingest_document(
            document_id=doc["id"],
            title=doc["title"],
            url=doc["url"],
            content=doc["content"],
            metadata=doc["metadata"]
        )
        
        return {"result": result, "document_id": doc_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting file: {str(e)}")