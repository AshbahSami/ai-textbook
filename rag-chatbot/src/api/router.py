"""
API router for the contextual RAG lookup service
"""
from fastapi import APIRouter, Depends, HTTPException, status
from typing import Optional, List
import logging
from ..models.request_models import ContextQueryRequest, ContextQueryResponse
from ..services.rag_service import RAGService
from ..services.chat_history_service import ChatHistoryService
from ..utils.exceptions import ValidationError
from ..config.database import get_db


logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize services
rag_service = RAGService()
chat_history_service = ChatHistoryService()


@router.post("/context-query", response_model=ContextQueryResponse)
async def context_query_endpoint(request: ContextQueryRequest):
    """
    Process a request for explaining selected text with provided context
    """
    try:
        # Validate request data
        if not request.selected_text.strip():
            raise ValidationError("selected_text cannot be empty")

        if not request.surrounding_context.strip():
            raise ValidationError("surrounding_context cannot be empty")

        if not request.full_page_url.strip():
            raise ValidationError("full_page_url cannot be empty")

        if not request.session_id.strip():
            raise ValidationError("session_id cannot be empty")

        # Validate session if needed
        # For now, we'll allow new session IDs, but we could require validation
        # if session management requires pre-existing sessions

        # Process the contextual query using the RAG service
        result = rag_service.process_contextual_query(
            session_id=request.session_id,
            selected_text=request.selected_text,
            full_page_url=request.full_page_url,
            surrounding_context=request.surrounding_context
        )

        # Create the response
        response = ContextQueryResponse(
            explanation=result.explanation,
            source_url=request.full_page_url,
            session_id=request.session_id,
            supplemental_context_used=result.supplemental_context_used
        )

        # Save the interaction to chat history
        chat_history_service.save_interaction(
            session_id=request.session_id,
            user_query=request.selected_text,
            context_used=request.surrounding_context,
            llm_response=result.explanation,
            source_url=request.full_page_url
        )

        logger.info(f"Processed context query for session {request.session_id}")
        return response

    except ValidationError:
        # Re-raise validation errors as they are already handled by our custom exception
        raise
    except Exception as e:
        logger.error(f"Error processing context query: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An internal error occurred while processing the query"
        )


@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "contextual-rag-lookup"}


@router.get("/sessions")
async def get_all_sessions():
    """
    Get all unique session IDs
    """
    try:
        sessions = chat_history_service.get_all_sessions()
        return {"sessions": sessions}
    except Exception as e:
        logger.error(f"Error retrieving all sessions: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving sessions"
        )


@router.get("/session/{session_id}")
async def get_session_history(session_id: str):
    """
    Get chat history for a specific session
    """
    try:
        # Validate the session exists
        if not chat_history_service.validate_session(session_id):
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Session {session_id} not found"
            )

        history = chat_history_service.get_session_history(session_id)
        return {
            "session_id": session_id,
            "history": history
        }
    except HTTPException:
        # Re-raise HTTP exceptions as they are already handled
        raise
    except Exception as e:
        logger.error(f"Error retrieving session history: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving session history"
        )


@router.get("/session/{session_id}/summary")
async def get_session_summary(session_id: str):
    """
    Get a summary of a specific session
    """
    try:
        # Validate the session exists
        if not chat_history_service.validate_session(session_id):
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Session {session_id} not found"
            )

        summary = chat_history_service.get_session_summary(session_id)
        return summary
    except HTTPException:
        # Re-raise HTTP exceptions as they are already handled
        raise
    except Exception as e:
        logger.error(f"Error retrieving session summary: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while retrieving session summary"
        )


@router.delete("/cleanup-sessions")
async def cleanup_sessions(days_to_keep: int = 30):
    """
    Clean up sessions older than the specified number of days
    """
    try:
        deleted_count = chat_history_service.cleanup_old_sessions(days_to_keep)
        return {
            "message": f"Cleaned up {deleted_count} old interactions",
            "days_to_keep": days_to_keep
        }
    except Exception as e:
        logger.error(f"Error cleaning up sessions: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="An error occurred while cleaning up sessions"
        )