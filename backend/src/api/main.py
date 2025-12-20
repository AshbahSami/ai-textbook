import os
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-based chatbot with documentation search capabilities",
    version="1.0.0"
)

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_HOST", "localhost"),
    port=os.getenv("QDRANT_PORT", 6333),
    api_key=os.getenv("QDRANT_API_KEY")
)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Import and include routes
from .routes.chat import router as chat_router
from .routes.ingest import router as ingest_router
from .routes.query import router as query_router

app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(ingest_router, prefix="/api/ingest", tags=["ingest"])
app.include_router(query_router, prefix="/api/query", tags=["query"])