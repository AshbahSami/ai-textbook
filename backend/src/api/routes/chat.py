from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import os
from ..services.chat import ChatService
from ..services.retrieval import RetrievalService
from qdrant_client import QdrantClient
from dotenv import load_dotenv
import openai
from langchain.prompts import ChatPromptTemplate
from langchain_community.chat_models import ChatOpenAI

# Load environment variables
load_dotenv()

# Initialize router
router = APIRouter()

# Initialize services
chat_service = ChatService()
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_HOST", "localhost"),
    port=os.getenv("QDRANT_PORT", 6333),
    api_key=os.getenv("QDRANT_API_KEY")
)
retrieval_service = RetrievalService(qdrant_client)

# Configure OpenAI client for Google Gemini
openai_client = openai.OpenAI(
    base_url=os.getenv("GEMINI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/"),
    api_key=os.getenv("GEMINI_API_KEY")
)

class ChatRequest(BaseModel):
    session_id: Optional[str] = None
    message: str
    user_id: Optional[str] = None
    highlighted_text: Optional[str] = None

class ChatResponse(BaseModel):
    session_id: str
    response: str
    sources: List[Dict[str, Any]]

@router.post("/message")
async def send_message(request: ChatRequest):
    try:
        # Create or validate session
        session_id = request.session_id
        if not session_id:
            session_id = chat_service.create_session(user_id=request.user_id)
        else:
            session = chat_service.get_session(session_id)
            if not session:
                raise HTTPException(status_code=404, detail="Session not found")
        
        # Add user message to session
        chat_service.add_message(
            session_id=session_id,
            sender_type="user",
            content=request.message
        )
        
        # Retrieve relevant documents based on the query
        query = request.message
        if request.highlighted_text:
            query = f"Context: {request.highlighted_text}\nQuestion: {request.message}"
        
        relevant_docs = retrieval_service.retrieve_relevant_documents(query, top_k=5)
        
        # Prepare context from retrieved documents
        context_str = "\n\n".join([doc["content_chunk"] for doc in relevant_docs])
        
        # Create prompt for the LLM
        if context_str:
            prompt = f"""
            Answer the question based on the provided context. If the answer is not in the context, say "I don't have enough information to answer that question based on the documentation."

            Context:
            {context_str}

            Question: {request.message}
            Answer:
            """
        else:
            prompt = f"""
            Answer the question based on your general knowledge. The user is asking about documentation, but no relevant documents were found.

            Question: {request.message}
            Answer:
            """
        
        # Generate response using Google Gemini via OpenAI-compatible endpoint
        response = openai_client.chat.completions.create(
            model="gemini-2.0-flash",  # Using Google Gemini 2.0 Flash
            messages=[
                {"role": "system", "content": "You are a helpful assistant that answers questions based on provided documentation. Be concise and accurate, and cite the sources when possible."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
            temperature=0.7
        )
        
        # Extract the response
        answer = response.choices[0].message.content
        
        # Add assistant message to session
        chat_service.add_message(
            session_id=session_id,
            sender_type="assistant",
            content=answer,
            context_sources=[doc["document_id"] for doc in relevant_docs]
        )
        
        # Prepare response
        response_data = ChatResponse(
            session_id=session_id,
            response=answer,
            sources=[{
                "document_id": doc["document_id"],
                "content_snippet": doc["content_chunk"][:200] + "..." if len(doc["content_chunk"]) > 200 else doc["content_chunk"],
                "score": doc["score"],
                "source_metadata": doc["source_metadata"]
            } for doc in relevant_docs]
        )
        
        return response_data
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat message: {str(e)}")

@router.get("/session/{session_id}")
async def get_session_history(session_id: str):
    try:
        messages = chat_service.get_session_history(session_id)
        return {"session_id": session_id, "messages": messages}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving session history: {str(e)}")

@router.post("/session")
async def create_session(user_id: Optional[str] = None):
    try:
        session_id = chat_service.create_session(user_id=user_id)
        return {"session_id": session_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating session: {str(e)}")