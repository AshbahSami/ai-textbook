# RAG Chatbot for Docusaurus

This project implements a Retrieval-Augmented Generation (RAG) chatbot for Docusaurus documentation sites. The chatbot allows users to ask questions about documentation content and receive AI-generated responses based on the indexed documentation.

## Architecture

The system consists of:
- **Backend API**: Python/FastAPI service that handles chat requests, document ingestion, and RAG processing
- **Vector Storage**: Qdrant for storing document embeddings
- **Database**: PostgreSQL (Neon) for storing chat sessions and metadata
- **Frontend**: React chat widget integrated into Docusaurus
- **LLM**: Google Gemini 2.0 Flash via OpenAI-compatible endpoint

## Prerequisites

- Python 3.11+
- Node.js 18+
- Docker and Docker Compose (for local Qdrant and Neon DB)
- Google API Key for Gemini 2.0 Flash

## Setup Instructions

### 1. Clone and Prepare the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set Up Environment Variables

Create a `.env` file in the root directory with the following variables:

```env
# Google Gemini API
GEMINI_API_KEY=your_gemini_api_key_here
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# Qdrant Configuration
QDRANT_HOST=localhost
QDRANT_PORT=6333
QDRANT_API_KEY=your_qdrant_api_key

# Neon DB Configuration
NEON_DB_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
BACKEND_API_URL=http://localhost:8000
```

### 3. Set Up Infrastructure with Docker

```bash
# Start Qdrant using Docker
docker-compose up -d
```

### 4. Set Up Backend

```bash
cd backend
pip install -r requirements.txt

# Run the backend server
python main.py
```

### 5. Set Up Documentation Ingestion

```bash
# Example: Ingest documentation from a URL
curl -X POST "http://localhost:8000/api/ingest/url" \
  -H "Content-Type: application/json" \
  -d '{"url": "https://yoursite.com/docs", "max_pages": 10}'
```

### 6. Set Up Frontend (Docusaurus Integration)

```bash
cd book-website
npm install

# Build and start the Docusaurus site
npm run build
npm run serve
# or for development
npm run start
```

## Usage

1. Navigate to your Docusaurus documentation site
2. You'll see a chat icon floating in the bottom-right corner
3. Click the icon to open the chat interface
4. Type your question about the documentation
5. The chatbot will retrieve relevant information and generate a response

## Features

- **Floating Chat Widget**: Appears on all pages of your Docusaurus site
- **Contextual Responses**: Answers based on indexed documentation content
- **Text Highlighting**: Select text on the page and ask questions about it
- **Session Management**: Maintains conversation history
- **Source Attribution**: Shows which documents informed the response

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /api/chat/session` - Create a new chat session
- `POST /api/chat/message` - Send a message and get a response
- `GET /api/chat/session/{session_id}` - Get session history
- `POST /api/ingest/documents` - Ingest multiple documents
- `POST /api/ingest/url` - Ingest content from a URL
- `POST /api/ingest/file` - Ingest content from a file upload
- `POST /api/query/search` - Search documents directly

## Development

To run the backend in development mode:

```bash
cd backend
python main.py
```

To run the Docusaurus site in development mode:

```bash
cd book-website
npm run start
```

## Troubleshooting

- Make sure all environment variables are properly set
- Verify that Qdrant is running and accessible
- Check that your Gemini API key has the necessary permissions
- Ensure the documentation content is properly ingested into the vector store