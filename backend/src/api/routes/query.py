from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Any
import os
from ..services.retrieval import RetrievalService
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
retrieval_service = RetrievalService(qdrant_client)

class QueryRequest(BaseModel):
    query: str
    top_k: int = 5

class QueryResponse(BaseModel):
    query: str
    results: List[Dict[str, Any]]

@router.post("/search")
async def search_documents(request: QueryRequest):
    try:
        results = retrieval_service.retrieve_relevant_documents(
            query=request.query,
            top_k=request.top_k
        )
        return QueryResponse(query=request.query, results=results)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error searching documents: {str(e)}")

@router.get("/document/{document_id}")
async def get_document_chunks(document_id: str):
    try:
        chunks = retrieval_service.retrieve_by_document_id(document_id)
        return {"document_id": document_id, "chunks": chunks}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving document: {str(e)}")