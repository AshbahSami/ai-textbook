from fastapi import APIRouter
from typing import Dict
import time
from ...utils.logging import get_logger
from ...services.retrieval import RAGRetrievalService
from ...services.chat import ChatSessionService
import os

logger = get_logger(__name__)
router = APIRouter()

retrieval_service = RAGRetrievalService()
chat_service = ChatSessionService()


@router.get("/health")
async def health_check() -> Dict:
    """
    Comprehensive health check for the application
    """
    start_time = time.time()
    
    # Check database connection
    db_healthy = False
    try:
        # Attempt to create a temporary session to test DB connection
        temp_session = chat_service.create_session(user_id="health_check")
        db_healthy = temp_session is not None
    except Exception as e:
        logger.error(f"Database health check failed: {str(e)}")
        db_healthy = False
    
    # Check Qdrant connection
    qdrant_healthy = False
    try:
        # Attempt a simple operation to test Qdrant connection
        collections = retrieval_service.qdrant_client.get_collections()
        qdrant_healthy = True
    except Exception as e:
        logger.error(f"Qdrant health check failed: {str(e)}")
        qdrant_healthy = False
    
    # Check Gemini API connection
    gemini_healthy = False
    try:
        # Attempt to generate a simple embedding to test Gemini connection
        test_embedding = retrieval_service.embed_text("health check")
        gemini_healthy = len(test_embedding) > 0
    except Exception as e:
        logger.error(f"Gemini API health check failed: {str(e)}")
        gemini_healthy = False
    
    # Calculate response time
    response_time = round((time.time() - start_time) * 1000, 2)  # in milliseconds
    
    # Overall health status
    overall_healthy = db_healthy and qdrant_healthy and gemini_healthy
    
    health_status = {
        "status": "healthy" if overall_healthy else "unhealthy",
        "timestamp": int(time.time()),
        "response_time_ms": response_time,
        "checks": {
            "database": {
                "status": "healthy" if db_healthy else "unhealthy",
                "message": "Database connection is working" if db_healthy else "Database connection failed"
            },
            "qdrant": {
                "status": "healthy" if qdrant_healthy else "unhealthy",
                "message": "Qdrant connection is working" if qdrant_healthy else "Qdrant connection failed"
            },
            "gemini_api": {
                "status": "healthy" if gemini_healthy else "unhealthy",
                "message": "Gemini API connection is working" if gemini_healthy else "Gemini API connection failed"
            }
        }
    }
    
    if overall_healthy:
        logger.info("Health check passed")
    else:
        logger.warning("Health check failed", extra=health_status)
    
    return health_status


@router.get("/ready")
async def readiness_check() -> Dict:
    """
    Readiness check for the application
    """
    # For now, use the same checks as health check
    # In a more complex system, this might check if all initialization is complete
    return await health_check()