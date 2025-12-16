from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # API Keys
    gemini_api_key: str
    qdrant_api_key: Optional[str] = None
    
    # Database
    database_url: str
    
    # Qdrant
    qdrant_host: str = "localhost"
    qdrant_port: int = 6333
    qdrant_collection_name: str = "rag-knowledge-base"
    
    # Application
    debug: bool = False
    app_name: str = "Contextual RAG Lookup Service"
    version: str = "0.0.1"
    api_prefix: str = "/api"
    
    class Config:
        env_file = ".env"
        case_sensitive = True


settings = Settings()