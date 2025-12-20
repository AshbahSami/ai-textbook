# Research Summary: RAG Chatbot for Docusaurus

**Feature**: RAG Chatbot for Docusaurus
**Date**: 2025-12-20
**Status**: Completed

## Overview

This document summarizes the research conducted to resolve technical unknowns and make key decisions for implementing a Retrieval-Augmented Generation (RAG) chatbot for Docusaurus documentation sites.

## Key Decisions Made

### 1. Documentation Ingestion Approach
**Decision**: Use a dedicated ingestion script that can crawl Docusaurus documentation or process static files
**Rationale**: The system needs to extract documentation content from Docusaurus sites to create vector embeddings. This can be done by either crawling the live site or processing the documentation files directly.
**Alternatives considered**: 
- Real-time crawling vs. batch processing
- Using Docusaurus plugins vs. external scripts
- Decision: External scripts provide more flexibility and better control over the ingestion process

### 2. Text Splitting Strategy
**Decision**: Use RecursiveCharacterTextSplitter for chunking documentation content
**Rationale**: This approach maintains semantic coherence while creating appropriately sized chunks for embedding.
**Alternatives considered**:
- Sentence-based splitting
- Paragraph-based splitting
- Custom splitting based on Docusaurus sections
- Decision: RecursiveCharacterTextSplitter provides the best balance of simplicity and effectiveness

### 3. Vector Storage Solution
**Decision**: Use Qdrant for vector embeddings storage
**Rationale**: Qdrant is specifically designed for vector similarity search and integrates well with the RAG pattern.
**Alternatives considered**:
- Pinecone
- Weaviate
- FAISS
- Decision: Qdrant chosen for its open-source nature, performance, and good Python client support

### 4. LLM Integration
**Decision**: Use OpenAI Agents SDK with Google Gemini 2.0 Flash via OpenAI-compatible endpoint
**Rationale**: The feature specification specifically requires using Google Gemini 2.0 Flash via the OpenAI-compatible base URL.
**Implementation approach**: Configure the OpenAI client with the custom base URL for Google's API.

### 5. Session Storage
**Decision**: Use Neon DB (PostgreSQL) for storing chat session history and user metadata
**Rationale**: PostgreSQL provides reliable, structured storage with ACID properties, and Neon offers a serverless PostgreSQL option.
**Alternatives considered**:
- MongoDB for document storage
- Redis for session caching
- Decision: PostgreSQL chosen for its reliability and structure for session data

### 6. Frontend Integration
**Decision**: Implement as a Docusaurus theme component using React
**Rationale**: This allows seamless integration with Docusaurus sites while maintaining the floating chat widget requirement.
**Implementation approach**: Create a React component that can be injected into Docusaurus pages via swizzling or as a plugin.

## Technical Unknowns Resolved

### 1. Docusaurus Integration Method
**Unknown**: How to best integrate the chat widget with Docusaurus
**Resolution**: The widget can be implemented as a React component and injected using either:
- Docusaurus swizzling (modifying the Footer component)
- Creating a custom plugin that injects the component globally
- Using the Root component approach
**Chosen approach**: Using Root component to ensure the widget appears on all pages without modifying core Docusaurus components

### 2. Text Highlighting Feature Implementation
**Unknown**: How to implement the text highlighting and question feature
**Resolution**: Use JavaScript to detect text selection on the page and pass the selected text to the chat interface
**Implementation approach**: Add an event listener for text selection that updates the chat interface with the selected text

### 3. Qdrant Collection Structure
**Unknown**: How to structure Qdrant collections for optimal retrieval
**Resolution**: Create collections with metadata fields for document source, section, and other relevant information for better context retrieval
**Implementation approach**: Each chunk will have metadata about its source document and position

### 4. Rate Limiting and Caching
**Unknown**: How to handle API rate limits and optimize performance
**Resolution**: Implement request caching and rate limiting to manage API calls to the LLM
**Implementation approach**: Use Redis or similar for caching responses and implement appropriate rate limiting

## Dependencies and Best Practices

### Backend Dependencies
- `openai`: For interacting with the Gemini API via OpenAI-compatible endpoint
- `qdrant-client`: For vector storage and retrieval
- `fastapi`: For creating the backend API
- `psycopg2-binary`: For PostgreSQL connectivity
- `python-dotenv`: For environment variable management
- `tiktoken`: For token counting (if needed for chunking)
- `beautifulsoup4` and `requests`: For web crawling (if crawling approach is used)

### Frontend Dependencies
- Standard Docusaurus dependencies (React, etc.)
- `@docusaurus/core`: For Docusaurus integration
- CSS libraries for styling the chat widget

### Architecture Patterns
- Repository pattern for data access
- Service layer pattern for business logic
- Factory pattern for creating different types of document processors
- Observer pattern for UI updates in the chat interface

## Implementation Phases

### Phase 1: Knowledge Ingestion
- Create `ingest.py` script to crawl Docusaurus `/docs` folder or live URL
- Use RecursiveCharacterTextSplitter to chunk content
- Embed chunks using Gemini's embedding model and store in Qdrant

### Phase 2: Agent Backend
- Initialize OpenAI Agents SDK with Gemini API key
- Configure agent with "Retriever Tool" that queries Qdrant
- Set up Neon DB connection for chat session history
- Create FastAPI endpoints for frontend communication

### Phase 3: Docusaurus Frontend
- Create React component `ChatWidget` positioned at bottom-right
- Implement toggle state for mini-chat window
- Integrate with backend API

### Phase 4: Integration
- Inject ChatWidget globally using Docusaurus Root component
- Test end-to-end functionality