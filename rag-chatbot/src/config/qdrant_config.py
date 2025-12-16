"""
Qdrant client setup module
"""
from qdrant_client import QdrantClient
from ..config.settings import settings


def get_qdrant_client():
    """
    Creates and returns a Qdrant client instance
    """
    if settings.qdrant_api_key:
        client = QdrantClient(
            host=settings.qdrant_host,
            port=settings.qdrant_port,
            api_key=settings.qdrant_api_key,
            https=True
        )
    else:
        # For local development without authentication
        client = QdrantClient(
            host=settings.qdrant_host,
            port=settings.qdrant_port
        )
    
    return client


# Create a global instance of the client
qdrant_client = get_qdrant_client()