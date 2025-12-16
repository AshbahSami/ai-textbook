"""
Main application entry point for the Contextual RAG Lookup service
"""
import logging
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .api.router import router
from .config.settings import settings


# Configure logging
logging.basicConfig(
    level=logging.DEBUG if settings.debug else logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handlers for the application
    """
    logger.info("Starting up the Contextual RAG Lookup service...")
    # Initialize resources on startup
    yield
    # Clean up resources on shutdown
    logger.info("Shutting down the Contextual RAG Lookup service...")


# Create FastAPI app with lifespan and metadata
app = FastAPI(
    title=settings.app_name,
    description="A service for contextual RAG lookup to explain selected text from documentation",
    version=settings.version,
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(router, prefix=settings.api_prefix)

@app.get("/")
async def root():
    """
    Root endpoint
    """
    return {
        "message": "Welcome to the Contextual RAG Lookup Service",
        "version": settings.version,
        "status": "running"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )