import hashlib
import logging
from typing import List, Dict, Any
from datetime import datetime
from qdrant_client import QdrantClient
from qdrant_client.http import models
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.embeddings import OpenAIEmbeddings
from pydantic import BaseModel
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DocumentIngestionService:
    def __init__(self, qdrant_client: QdrantClient, collection_name: str = "docs_collection"):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            length_function=len,
        )
        
        # Initialize OpenAI embeddings (using Google Gemini via OpenAI-compatible endpoint)
        self.embeddings = OpenAIEmbeddings(
            openai_api_base=os.getenv("GEMINI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/"),
            openai_api_key=os.getenv("GEMINI_API_KEY"),
            model="models/embedding-001"  # Using Gemini embedding model
        )
        
        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """Create Qdrant collection if it doesn't exist"""
        try:
            self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create collection with vector configuration
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),  # Adjust size as needed
            )
            logger.info(f"Created collection {self.collection_name}")

    def _calculate_content_hash(self, content: str) -> str:
        """Calculate hash of content to detect changes"""
        return hashlib.sha256(content.encode()).hexdigest()

    def _split_document(self, content: str) -> List[str]:
        """Split document content into chunks"""
        chunks = self.text_splitter.split_text(content)
        return chunks

    def _generate_embeddings(self, chunks: List[str]) -> List[List[float]]:
        """Generate embeddings for chunks"""
        embeddings = self.embeddings.embed_documents(chunks)
        return embeddings

    def _store_embeddings(self, document_id: str, chunks: List[str], embeddings: List[List[float]], metadata: Dict[str, Any] = None):
        """Store embeddings in Qdrant"""
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = f"{document_id}_chunk_{i}"
            payload = {
                "document_id": document_id,
                "content_chunk": chunk,
                "chunk_index": i,
                "source_metadata": metadata or {}
            }
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload
                )
            )
        
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points
        )
        logger.info(f"Stored {len(points)} embeddings for document {document_id}")

    def ingest_document(self, document_id: str, title: str, url: str, content: str, metadata: Dict[str, Any] = None) -> bool:
        """Ingest a single document into the vector store"""
        try:
            # Calculate content hash
            source_hash = self._calculate_content_hash(content)
            
            # Check if document already exists with same hash
            existing_points = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchValue(value=document_id)
                        )
                    ]
                ),
                limit=1
            )
            
            # If document exists and hash matches, skip ingestion
            if existing_points[0]:
                logger.info(f"Document {document_id} already exists, checking for changes...")
                # For simplicity, we'll update the document even if it exists
                # In a real implementation, you'd compare the source_hash
            
            # Split document into chunks
            chunks = self._split_document(content)
            logger.info(f"Split document into {len(chunks)} chunks")
            
            # Generate embeddings for chunks
            embeddings = self._generate_embeddings(chunks)
            logger.info(f"Generated {len(embeddings)} embeddings")
            
            # Prepare metadata
            doc_metadata = {
                "title": title,
                "url": url,
                "source_hash": source_hash,
                "created_at": datetime.now().isoformat()
            }
            if metadata:
                doc_metadata.update(metadata)
            
            # Store embeddings in Qdrant
            self._store_embeddings(document_id, chunks, embeddings, doc_metadata)
            
            logger.info(f"Successfully ingested document {document_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error ingesting document {document_id}: {str(e)}")
            return False

    def ingest_documents(self, documents: List[Dict[str, Any]]) -> Dict[str, bool]:
        """Ingest multiple documents"""
        results = {}
        for doc in documents:
            doc_id = doc.get('id')
            results[doc_id] = self.ingest_document(
                document_id=doc_id,
                title=doc.get('title', ''),
                url=doc.get('url', ''),
                content=doc.get('content', ''),
                metadata=doc.get('metadata', {})
            )
        return results