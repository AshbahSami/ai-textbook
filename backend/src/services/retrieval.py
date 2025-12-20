import logging
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from langchain_community.embeddings import OpenAIEmbeddings
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RetrievalService:
    def __init__(self, qdrant_client: QdrantClient, collection_name: str = "docs_collection"):
        self.qdrant_client = qdrant_client
        self.collection_name = collection_name
        
        # Initialize OpenAI embeddings (using Google Gemini via OpenAI-compatible endpoint)
        self.embeddings = OpenAIEmbeddings(
            openai_api_base=os.getenv("GEMINI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/"),
            openai_api_key=os.getenv("GEMINI_API_KEY"),
            model="models/embedding-001"  # Using Gemini embedding model
        )

    def _generate_query_embedding(self, query: str) -> List[float]:
        """Generate embedding for the query"""
        embedding = self.embeddings.embed_query(query)
        return embedding

    def retrieve_relevant_documents(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """Retrieve relevant documents based on the query"""
        try:
            # Generate embedding for the query
            query_embedding = self._generate_query_embedding(query)
            
            # Search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )
            
            # Format results
            results = []
            for hit in search_results:
                result = {
                    "document_id": hit.payload.get("document_id"),
                    "content_chunk": hit.payload.get("content_chunk"),
                    "score": hit.score,
                    "source_metadata": hit.payload.get("source_metadata", {}),
                    "chunk_index": hit.payload.get("chunk_index")
                }
                results.append(result)
            
            logger.info(f"Retrieved {len(results)} relevant documents for query")
            return results
            
        except Exception as e:
            logger.error(f"Error retrieving documents for query '{query}': {str(e)}")
            return []

    def retrieve_by_document_id(self, document_id: str) -> List[Dict[str, Any]]:
        """Retrieve all chunks for a specific document"""
        try:
            results = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchValue(value=document_id)
                        )
                    ]
                )
            )
            
            chunks = []
            for point, _ in results:
                chunk = {
                    "document_id": point.payload.get("document_id"),
                    "content_chunk": point.payload.get("content_chunk"),
                    "chunk_index": point.payload.get("chunk_index"),
                    "source_metadata": point.payload.get("source_metadata", {})
                }
                chunks.append(chunk)
            
            logger.info(f"Retrieved {len(chunks)} chunks for document {document_id}")
            return chunks
            
        except Exception as e:
            logger.error(f"Error retrieving document {document_id}: {str(e)}")
            return []