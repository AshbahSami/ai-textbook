# Quickstart Guide: RAG Chatbot for Docusaurus

**Feature**: RAG Chatbot for Docusaurus
**Date**: 2025-12-20
**Status**: Draft

## Overview

This guide provides instructions to quickly set up and run the RAG Chatbot for Docusaurus. This system allows users to ask questions about documentation content through a chat interface, with responses generated based on the indexed documentation.

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus)
- Docker and Docker Compose (for local Qdrant and Neon DB)
- Google API Key for Gemini 2.0 Flash
- Access to Docusaurus documentation site content

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
QDRANT_API_KEY=your_qdrant_api_key  # If using cloud version

# Neon DB Configuration
NEON_DB_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# Backend Configuration
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
```

### 3. Set Up Infrastructure with Docker

```bash
# Start Qdrant using Docker
docker-compose up -d
```

Example `docker-compose.yml`:
```yaml
version: '3.9'
services:
  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
      - "6334:6334"
    volumes:
      - ./qdrant_data:/qdrant/storage
    environment:
      - QDRANT_API_KEY=${QDRANT_API_KEY:-""}
```

### 4. Set Up Backend

```bash
cd backend
pip install -r requirements.txt

# Run the backend server
python -m src.api.main
```

### 5. Set Up Documentation Ingestion

```bash
# Ingest documentation from a local directory
python -m src.cli.ingest_cli --source-type directory --source-path /path/to/docs --collection-name my-docs

# Or ingest from a URL
python -m src.cli.ingest_cli --source-type url --source-path https://example.com/docs --collection-name my-docs
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

## Running the System

### 1. Start the Backend API

```bash
cd backend
python -m src.api.main
```

### 2. Start the Docusaurus Site

```bash
cd book-website
npm run start
```

### 3. Verify Components

1. Backend API should be running at `http://localhost:8000`
2. Docusaurus site should be running at `http://localhost:3000`
3. Chat widget should appear as a floating button in the bottom-right corner

## Using the Chat Interface

1. Navigate to your Docusaurus documentation site
2. Click the floating chat widget in the bottom-right corner
3. Type your question about the documentation
4. Receive an AI-generated response based on the indexed documentation content
5. Optionally, highlight text on the page and ask questions about it

## Testing the System

### Backend API Tests

```bash
cd backend
pytest tests/unit/
pytest tests/integration/
```

### Frontend Tests

```bash
cd book-website
npm test
```

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your GEMINI_API_KEY is valid and has the necessary permissions
2. **Qdrant Connection**: Verify that Qdrant is running and accessible at the configured host/port
3. **Documentation Ingestion**: Check that the documentation source is accessible and properly formatted

### Verification Commands

```bash
# Test backend connectivity
curl http://localhost:8000/health

# Test documentation ingestion
python -c "from src.services.ingestion import test_ingestion; test_ingestion()"

# Check Qdrant collections
curl http://localhost:6333/collections
```

## Next Steps

1. Customize the chat widget styling to match your documentation site
2. Configure additional documentation sources
3. Set up monitoring and logging for production use
4. Implement rate limiting and caching for production deployment