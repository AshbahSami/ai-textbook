import pytest
from unittest.mock import Mock, patch, MagicMock
from src.services.rag_service import RAGService
from src.models.request_models import ContextualQuery, QueryResult


class TestRAGService:
    """
    Unit tests for RAGService
    """
    
    @pytest.fixture
    def rag_service(self):
        with patch('src.services.rag_service.genai') as mock_genai, \
             patch('src.services.rag_service.QdrantService') as mock_qdrant:
            
            # Mock the genai configuration and model
            mock_genai.configure = Mock()
            mock_genai.GenerativeModel = Mock()
            mock_model_instance = Mock()
            mock_model_instance.generate_content = Mock()
            mock_genai.GenerativeModel.return_value = mock_model_instance
            
            service = RAGService()
            service.gemini_model = mock_model_instance
            service.qdrant_service = mock_qdrant.return_value
            
            return service
    
    def test_process_contextual_query_success(self, rag_service):
        """
        Test successful processing of a contextual query
        """
        # Mock the Gemini response
        mock_response = Mock()
        mock_response.text = "This is a generated explanation."
        rag_service.gemini_model.generate_content.return_value = mock_response
        
        # Mock the Qdrant service to return empty supplemental context
        rag_service.qdrant_service._retrieve_supplemental_context = Mock(return_value=[])
        
        # Call the method under test
        result = rag_service.process_contextual_query(
            session_id="test-session",
            selected_text="test text",
            full_page_url="https://example.com",
            surrounding_context="This is the surrounding context."
        )
        
        # Assertions
        assert isinstance(result, QueryResult)
        assert result.explanation == "This is a generated explanation."
        assert result.supplemental_context_used is False
        assert result.context_quality_score is not None
        assert result.processing_time_ms >= 0
        
        # Verify the Gemini model was called
        rag_service.gemini_model.generate_content.assert_called_once()
    
    def test_evaluate_context_quality(self, rag_service):
        """
        Test context quality evaluation
        """
        selected_text = "machine learning"
        primary_context = "Machine learning is a subset of artificial intelligence."
        supplemental_context = ["AI involves neural networks."]
        explanation = "Machine learning is a subset of artificial intelligence that involves neural networks."
        
        quality_score = rag_service._evaluate_context_quality(
            selected_text, primary_context, supplemental_context, explanation
        )
        
        # The quality should be between 0 and 1
        assert 0 <= quality_score <= 1
    
    def test_construct_prompt(self, rag_service):
        """
        Test prompt construction
        """
        selected_text = "RAG pipeline"
        primary_context = "The RAG pipeline retrieves documents."
        supplemental_context = ["Documents are stored in Qdrant."]
        
        prompt = rag_service._construct_prompt(
            selected_text, primary_context, supplemental_context
        )
        
        # Check that the prompt contains all expected parts
        assert selected_text in prompt
        assert primary_context in prompt
        assert supplemental_context[0] in prompt
    
    def test_retrieve_supplemental_context(self, rag_service):
        """
        Test supplemental context retrieval
        """
        # In our implementation, this returns an empty list as a placeholder
        result = rag_service._retrieve_supplemental_context("test query")
        assert result == []