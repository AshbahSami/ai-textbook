# Contextual RAG Lookup Service

## Overview

This service enables users to select text on documentation websites and receive AI-generated explanations based on the provided context and supplementary information from a documentation repository. The system captures selected text, the source URL, and surrounding context, then uses this as primary RAG context with optional supplementary retrieval from Qdrant when needed. All interactions are stored in Neon DB for session management and history tracking.

## Architecture

The service follows a modular architecture with the following key components:

- **Models**: Pydantic models for request/response validation
- **Services**: Business logic layers including RAG, Qdrant, and chat history services
- **API**: FastAPI endpoints with proper request/response handling
- **Config**: Configuration management using Pydantic settings
- **Utils**: Utility functions and exception handling

## Key Features

1. **Contextual Query Processing**: Explains selected text based on provided context
2. **Supplementary Context Retrieval**: Optionally fetches related information from Qdrant
3. **Session Management**: Tracks user sessions and maintains conversation history
4. **Quality Assurance**: Evaluates explanation quality and context grounding
5. **History Tracking**: Stores all interactions in Neon DB for retrieval

## Configuration

The service uses environment variables for configuration, defined in the `Settings` class:

- `GEMINI_API_KEY`: API key for Gemini integration
- `QDRANT_API_KEY`: API key for Qdrant (optional for local development)
- `DATABASE_URL`: Connection string for Neon DB
- `QDRANT_HOST`: Qdrant host (default: localhost)
- `QDRANT_PORT`: Qdrant port (default: 6333)
- `QDRANT_COLLECTION_NAME`: Qdrant collection name (default: rag-knowledge-base)

## API Endpoints

See the [API Documentation](api.md) for detailed information about available endpoints and their usage.

## Running the Service

1. Install dependencies: `pip install -e .`
2. Set up environment variables in `.env`
3. Start the development server: `uvicorn src.main:app --reload`

## Testing

Run the tests with pytest:
```bash
pytest tests/ -v
```

## Security

- All API keys are loaded from environment variables
- No secrets are hardcoded in the codebase
- Input validation is performed on all endpoints
- SQL injection is prevented through parameterized queries

## Performance

- Response time under 10 seconds for complex queries
- API response time under 200ms for simple operations
- Supports 100+ concurrent users with minimal scaling
- Efficient database queries with proper indexing