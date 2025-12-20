# Data Model: RAG Chatbot for Docusaurus

**Feature**: RAG Chatbot for Docusaurus
**Date**: 2025-12-20
**Status**: Draft

## Overview

This document defines the data models for the RAG Chatbot for Docusaurus, including the entities, their attributes, relationships, and validation rules.

## Entities

### 1. Documentation Content

**Description**: Represents the text and structured information from the Docusaurus site that gets indexed and retrieved

**Attributes**:
- `id`: Unique identifier for the document (string, required)
- `title`: Title of the document (string, required)
- `url`: URL path of the document in the Docusaurus site (string, required)
- `content`: The main text content of the document (string, required)
- `metadata`: Additional metadata associated with the document (JSON object, optional)
- `created_at`: Timestamp when the document was indexed (datetime, required)
- `updated_at`: Timestamp when the document was last updated (datetime, required)
- `source_hash`: Hash of the source content to detect changes (string, required)

**Validation Rules**:
- `id` must be unique across all documents
- `title` must not be empty
- `url` must be a valid relative path
- `content` must not be empty
- `created_at` must be a valid timestamp
- `source_hash` must be a valid hash string

### 2. Vector Embedding

**Description**: Mathematical representations of documentation chunks stored in Qdrant for similarity search

**Attributes**:
- `id`: Unique identifier for the embedding (string, required)
- `document_id`: Reference to the parent Documentation Content (string, required)
- `content_chunk`: The text chunk that was embedded (string, required)
- `embedding_vector`: The vector representation of the content chunk (array of floats, required)
- `chunk_metadata`: Additional metadata for the chunk (JSON object, optional)
- `created_at`: Timestamp when the embedding was created (datetime, required)

**Validation Rules**:
- `id` must be unique across all embeddings
- `document_id` must reference an existing Documentation Content
- `content_chunk` must not be empty
- `embedding_vector` must be a valid array of floats with consistent dimensions
- `created_at` must be a valid timestamp

**Relationships**:
- Belongs to one Documentation Content
- Multiple Vector Embeddings can belong to one Documentation Content

### 3. Chat Session

**Description**: Represents a user's conversation history with timestamps and metadata

**Attributes**:
- `session_id`: Unique identifier for the chat session (string, required)
- `user_id`: Identifier for the user (string, optional)
- `created_at`: Timestamp when the session started (datetime, required)
- `updated_at`: Timestamp when the session was last updated (datetime, required)
- `is_active`: Whether the session is currently active (boolean, default: true)
- `metadata`: Additional metadata about the session (JSON object, optional)

**Validation Rules**:
- `session_id` must be unique across all sessions
- `created_at` must be a valid timestamp
- `updated_at` must be a valid timestamp
- `is_active` must be a boolean value

### 4. Chat Message

**Description**: Represents a single message in a chat conversation

**Attributes**:
- `message_id`: Unique identifier for the message (string, required)
- `session_id`: Reference to the parent Chat Session (string, required)
- `sender_type`: Type of sender (enum: 'user' or 'assistant', required)
- `content`: The text content of the message (string, required)
- `timestamp`: When the message was sent (datetime, required)
- `context_sources`: List of documentation sources used in generating the response (array of strings, optional)
- `metadata`: Additional metadata about the message (JSON object, optional)

**Validation Rules**:
- `message_id` must be unique across all messages
- `session_id` must reference an existing Chat Session
- `sender_type` must be either 'user' or 'assistant'
- `content` must not be empty
- `timestamp` must be a valid timestamp

**Relationships**:
- Belongs to one Chat Session
- Multiple Chat Messages can belong to one Chat Session

### 5. User Query

**Description**: The text input from users requesting information from the documentation

**Attributes**:
- This entity is primarily represented as the user's message in the Chat Message entity
- The query itself is the `content` field in a Chat Message where `sender_type` is 'user'

### 6. User Metadata

**Description**: Additional information about users interacting with the chatbot

**Attributes**:
- `user_id`: Unique identifier for the user (string, required)
- `session_count`: Number of sessions this user has had (integer, default: 0)
- `first_seen`: When this user first interacted with the system (datetime, required)
- `last_seen`: When this user last interacted with the system (datetime, required)
- `preferences`: User preferences (JSON object, optional)

**Validation Rules**:
- `user_id` must be unique across all users
- `session_count` must be a non-negative integer
- `first_seen` and `last_seen` must be valid timestamps

## Relationships

```
Documentation Content (1) <---> (Many) Vector Embedding
Documentation Content (1) <---> (Many) Chat Message (via context_sources)

Chat Session (1) <---> (Many) Chat Message
Chat Session (1) <---> (Many) User Metadata (via user_id)
```

## State Transitions

### Chat Session
- `is_active` can transition from `true` to `false` when a session is ended
- A session becomes inactive either by user action or after a period of inactivity