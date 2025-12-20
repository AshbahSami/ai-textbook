from typing import List
from qdrant_client.http import models
from .utils.qdrant import qdrant_util


class QdrantCollectionManager:
    """
    Manager class for handling Qdrant collection operations specific to documentation
    """
    
    def __init__(self):
        self.default_collection_name = "documentation_embeddings"
    
    def ensure_documentation_collection(self, collection_name: str = None) -> bool:
        """
        Ensure the documentation embeddings collection exists
        """
        collection_name = collection_name or self.default_collection_name
        
        if qdrant_util.collection_exists(collection_name):
            return True
        
        try:
            qdrant_util.create_collection(collection_name)
            return True
        except Exception as e:
            print(f"Error creating collection {collection_name}: {str(e)}")
            return False
    
    def create_point_struct(self, point_id: str, vector: List[float], payload: dict) -> models.PointStruct:
        """
        Create a PointStruct for Qdrant
        """
        return models.PointStruct(
            id=point_id,
            vector=vector,
            payload=payload
        )
    
    def batch_upsert_embeddings(self, collection_name: str, points: List[models.PointStruct]) -> bool:
        """
        Batch upsert embedding points to Qdrant
        """
        try:
            qdrant_util.upsert_points(collection_name, points)
            return True
        except Exception as e:
            print(f"Error upserting embeddings to collection {collection_name}: {str(e)}")
            return False
    
    def search_similar(self, collection_name: str, query_vector: List[float], top_k: int = 5):
        """
        Search for similar embeddings in the collection
        """
        return qdrant_util.search(collection_name, query_vector, top_k)
    
    def delete_document_embeddings(self, collection_name: str, document_id: str) -> bool:
        """
        Delete all embeddings associated with a specific document
        """
        try:
            # In a real implementation, we would need to search for points with a specific document_id
            # and then delete them. For now, this is a placeholder.
            print(f"Deleting embeddings for document {document_id} in collection {collection_name}")
            return True
        except Exception as e:
            print(f"Error deleting document embeddings: {str(e)}")
            return False


# Global instance for easy access
collection_manager = QdrantCollectionManager()