"""
Qdrant service for vector search operations
"""
import logging
from typing import List, Optional
from qdrant_client.http import models
from ..config.qdrant_config import qdrant_client
from ..config.settings import settings


class QdrantService:
    """
    Service for handling Qdrant vector search operations
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.collection_name = settings.qdrant_collection_name
        self._ensure_collection_exists()
    
    def _ensure_collection_exists(self):
        """
        Ensure the required collection exists in Qdrant
        """
        try:
            # Check if collection exists
            qdrant_client.get_collection(self.collection_name)
            self.logger.info(f"Collection '{self.collection_name}' already exists")
        except Exception:
            # Create collection if it doesn't exist
            # Assuming 768 dimensions for Gemini embeddings, adjust as needed
            qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
            )
            self.logger.info(f"Created collection '{self.collection_name}'")
    
    def search_similar(
        self,
        query_vector: List[float],
        top_k: int = 5,
        filters: Optional[dict] = None
    ) -> List[dict]:
        """
        Search for similar vectors in the collection
        """
        try:
            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )
                if filter_conditions:
                    qdrant_filters = models.Filter(must=filter_conditions)
            
            # Perform search
            results = qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                query_filter=qdrant_filters
            )
            
            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    'id': result.id,
                    'payload': result.payload,
                    'score': result.score,
                    'vector': result.vector
                })
            
            self.logger.info(f"Found {len(formatted_results)} similar results")
            return formatted_results
        except Exception as e:
            self.logger.error(f"Error searching similar vectors: {str(e)}")
            raise e
    
    def add_document(
        self,
        doc_id: str,
        vector: List[float],
        payload: dict,
        collection_name: Optional[str] = None
    ) -> bool:
        """
        Add a document vector to the collection
        """
        try:
            collection = collection_name or self.collection_name
            
            qdrant_client.upsert(
                collection_name=collection,
                points=[
                    models.PointStruct(
                        id=doc_id,
                        vector=vector,
                        payload=payload
                    )
                ]
            )
            
            self.logger.info(f"Added document {doc_id} to collection {collection}")
            return True
        except Exception as e:
            self.logger.error(f"Error adding document {doc_id}: {str(e)}")
            raise e
    
    def batch_add_documents(
        self,
        doc_ids: List[str],
        vectors: List[List[float]],
        payloads: List[dict],
        collection_name: Optional[str] = None
    ) -> bool:
        """
        Add multiple document vectors to the collection
        """
        try:
            collection = collection_name or self.collection_name
            
            points = []
            for i, (doc_id, vector, payload) in enumerate(zip(doc_ids, vectors, payloads)):
                points.append(
                    models.PointStruct(
                        id=doc_id,
                        vector=vector,
                        payload=payload
                    )
                )
            
            qdrant_client.upsert(
                collection_name=collection,
                points=points
            )
            
            self.logger.info(f"Added {len(doc_ids)} documents to collection {collection}")
            return True
        except Exception as e:
            self.logger.error(f"Error adding batch of documents: {str(e)}")
            raise e