# Contextual RAG Lookup Service

This service enables users to select text on documentation websites and receive AI-generated explanations based on the provided context and supplementary information from a documentation repository.

## Features

- Context-aware text explanation using RAG (Retrieval-Augmented Generation)
- Qdrant vector search for supplementary context
- Neon DB for session management and chat history
- FastAPI-based API with proper request/response validation
- Quality assurance with context grounding validation
- Session management and history tracking

## Architecture

The service follows a modular architecture with the following key components:

- **Models**: Pydantic models for request/response validation
- **Services**: Business logic layers including RAG, Qdrant, and chat history services
- **API**: FastAPI endpoints with proper request/response handling
- **Config**: Configuration management using Pydantic settings
- **Utils**: Utility functions and exception handling

## Prerequisites

- Python 3.11+
- Access to required API keys (GEMINI_API_KEY, QDRANT_API_KEY, DATABASE_URL)

## Installation

1. Clone the repository
2. Create a virtual environment: `python -m venv venv`
3. Activate the virtual environment: `source venv/bin/activate` (Linux/Mac) or `venv\Scripts\activate` (Windows)
4. Install dependencies: `pip install -e .`

## Environment Variables

Create a `.env` file with the following variables:

```env
GEMINI_API_KEY=your_gemini_api_key
QDRANT_API_KEY=your_qdrant_api_key  # Optional for local development
DATABASE_URL=postgresql://username:password@localhost:5432/your_db_name
QDRANT_HOST=localhost
QDRANT_PORT=6333
QDRANT_COLLECTION_NAME=rag-knowledge-base
DEBUG=true  # optional
```

## Running the Service

Start the development server:

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at:
- Main endpoint: `http://localhost:8000/api/context-query`
- API docs: `http://localhost:8000/docs`
- Alternative docs: `http://localhost:8000/redoc`

## API Endpoints

### POST `/api/context-query`

Request explanation for selected text with provided context.

Example request:
```json
{
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "selected_text": "The RAG pipeline processes queries",
  "full_page_url": "https://example.com/docs/rag-introduction",
  "surrounding_context": "Our system uses a RAG (Retrieval-Augmented Generation) pipeline to provide accurate responses to user queries. The RAG pipeline processes queries by retrieving relevant documents and generating responses."
}
```

Response:
```json
{
  "explanation": "The RAG pipeline refers to Retrieval-Augmented Generation...",
  "source_url": "https://example.com/docs/rag-introduction",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "supplemental_context_used": true
}
```

### GET `/api/sessions`

Get all unique session IDs.

### GET `/api/session/{session_id}`

Get chat history for a specific session.

### GET `/api/session/{session_id}/summary`

Get a summary of a specific session.

### DELETE `/api/cleanup-sessions`

Clean up sessions older than the specified number of days (default: 30).

### GET `/health`

Health check endpoint.

## Running Tests

Run the tests with pytest:
```bash
pytest tests/ -v
```

## Documentation

- API Documentation: [docs/api.md](docs/api.md)
- Architecture: [docs/index.md](docs/index.md)

## Security

- All API keys are loaded from environment variables
- Input validation is performed on all endpoints
- SQL injection is prevented through parameterized queries
- Rate limiting is implemented to prevent abuse

## Performance

- Response time under 10 seconds for complex queries
- API response time under 200ms for simple operations
- Supports 100+ concurrent users with minimal scaling
- Efficient database queries with proper indexing