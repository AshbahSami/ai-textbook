# API Documentation for Contextual RAG Lookup

## Overview

This API enables users to select text on documentation websites and receive AI-generated explanations based on the provided context and supplementary information from a documentation repository.

## Authentication

This API uses API key authentication. Include your API key in the `X-API-Key` header for all requests.

## Endpoints

### POST /api/context-query

Request an explanation for selected text with provided context.

#### Request Body

```json
{
  "session_id": "string (UUID format)",
  "selected_text": "string",
  "full_page_url": "string (URL)",
  "surrounding_context": "string"
}
```

#### Response

```json
{
  "explanation": "string",
  "source_url": "string",
  "session_id": "string",
  "supplemental_context_used": "boolean (optional)"
}
```

#### Example Request

```bash
curl -X POST "http://localhost:8000/api/context-query" \
  -H "Content-Type: application/json" \
  -H "X-API-Key: YOUR_API_KEY" \
  -d '{
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "selected_text": "The RAG pipeline processes queries",
    "full_page_url": "https://example.com/docs/rag-introduction",
    "surrounding_context": "Our system uses a RAG (Retrieval-Augmented Generation) pipeline to provide accurate responses to user queries. The RAG pipeline processes queries by retrieving relevant documents and generating responses."
  }'
```

### GET /api/sessions

Get all unique session IDs.

#### Response

```json
{
  "sessions": ["string"]
}
```

### GET /api/session/{session_id}

Get chat history for a specific session.

#### Response

```json
{
  "session_id": "string",
  "history": [
    {
      "id": "integer",
      "user_query": "string",
      "llm_response": "string",
      "source_url": "string",
      "timestamp": "datetime"
    }
  ]
}
```

### GET /api/session/{session_id}/summary

Get a summary of a specific session.

#### Response

```json
{
  "session_id": "string",
  "interaction_count": "integer",
  "first_interaction": "datetime",
  "last_interaction": "datetime"
}
```

### DELETE /api/cleanup-sessions

Clean up sessions older than the specified number of days.

#### Query Parameters

- `days_to_keep` (optional, default: 30)

#### Response

```json
{
  "message": "string",
  "days_to_keep": "integer"
}
```

### GET /api/health

Health check endpoint.

#### Response

```json
{
  "status": "healthy",
  "service": "contextual-rag-lookup"
}
```