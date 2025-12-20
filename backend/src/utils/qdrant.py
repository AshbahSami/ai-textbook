from qdrant_client import QdrantClient
from qdrant_client.http import models
import os
from typing import List, Dict, Any, Optional


class QdrantClientUtil:
    """
    Utility class for managing Qdrant vector database operations
    """
    
    def __init__(self):
        self.client = QdrantClient(
            host=os.getenv("QDRANT_HOST", "localhost"),
            port=int(os.getenv("QDRANT_PORT", 6333)),
            api_key=os.getenv("QDRANT_API_KEY")
        )
    
    def create_collection(self, collection_name: str, vector_size: int = 768):
        """
        Create a new collection in Qdrant
        """
        self.client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
        )
    
    def collection_exists(self, collection_name: str) -> bool:
        """
        Check if a collection exists in Qdrant
        """
        try:
            self.client.get_collection(collection_name)
            return True
        except:
            return False
    
    def upsert_points(self, collection_name: str, points: List[models.PointStruct]):
        """
        Upsert (insert or update) points in a collection
        """
        self.client.upsert(
            collection_name=collection_name,
            points=points
        )
    
    def search(self, collection_name: str, query_vector: List[float], limit: int = 10):
        """
        Search for similar vectors in a collection
        """
        return self.client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=limit
        )
    
    def delete_points(self, collection_name: str, point_ids: List[str]):
        """
        Delete points from a collection by their IDs
        """
        self.client.delete(
            collection_name=collection_name,
            points_selector=models.PointIdsList(
                points=point_ids
            )
        )
    
    def get_points_by_ids(self, collection_name: str, point_ids: List[str]):
        """
        Retrieve points from a collection by their IDs
        """
        return self.client.retrieve(
            collection_name=collection_name,
            ids=point_ids
        )
    
    def get_all_collections(self):
        """
        Get a list of all collections
        """
        return self.client.get_collections()


# Global instance for easy access
qdrant_util = QdrantClientUtil()