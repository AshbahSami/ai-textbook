import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
from src.main import app
from src.models.request_models import ContextQueryRequest


client = TestClient(app)


class TestAPIEndpoints:
    """
    Tests for API endpoints
    """
    
    @patch('src.api.router.rag_service')
    @patch('src.api.router.chat_history_service')
    def test_context_query_endpoint_success(self, mock_chat_history_service, mock_rag_service):
        """
        Test successful context query endpoint
        """
        # Mock the RAG service response
        mock_result = Mock()
        mock_result.explanation = "Test explanation"
        mock_result.supplemental_context_used = True
        mock_rag_service.process_contextual_query.return_value = mock_result
        
        # Mock the chat history service
        mock_chat_history_service.save_interaction.return_value = 1
        
        # Prepare test data
        test_request = ContextQueryRequest(
            session_id="test-session",
            selected_text="test text",
            full_page_url="https://example.com",
            surrounding_context="This is surrounding context."
        )
        
        # Make the request
        response = client.post("/api/context-query", json=test_request.model_dump())
        
        # Assertions
        assert response.status_code == 200
        response_data = response.json()
        assert response_data["explanation"] == "Test explanation"
        assert response_data["session_id"] == "test-session"
        assert response_data["supplemental_context_used"] is True
    
    @patch('src.api.router.rag_service')
    def test_context_query_endpoint_validation_error(self, mock_rag_service):
        """
        Test context query endpoint with validation error
        """
        # Prepare test data with invalid empty selected_text
        test_request = {
            "session_id": "test-session", 
            "selected_text": "",  # This should cause validation error
            "full_page_url": "https://example.com",
            "surrounding_context": "This is surrounding context."
        }
        
        # Make the request
        response = client.post("/api/context-query", json=test_request)
        
        # Assertions - Should return 422 for validation error
        assert response.status_code == 422
    
    def test_health_check_endpoint(self):
        """
        Test health check endpoint
        """
        # Make the request
        response = client.get("/health")
        
        # Assertions
        assert response.status_code == 200
        response_data = response.json()
        assert response_data["status"] == "healthy"
        assert response_data["service"] == "contextual-rag-lookup"
    
    @patch('src.api.router.chat_history_service')
    def test_get_session_history_success(self, mock_chat_history_service):
        """
        Test getting session history
        """
        # Mock the service response
        mock_history = [
            {
                "id": 1,
                "user_query": "test query",
                "llm_response": "test response",
                "source_url": "https://example.com",
                "timestamp": "2023-01-01T00:00:00"
            }
        ]
        mock_chat_history_service.get_session_history.return_value = mock_history
        mock_chat_history_service.validate_session.return_value = True
        
        # Make the request
        response = client.get("/api/session/test-session")
        
        # Assertions
        assert response.status_code == 200
        response_data = response.json()
        assert response_data["session_id"] == "test-session"
        assert len(response_data["history"]) == 1
    
    @patch('src.api.router.chat_history_service')
    def test_get_session_history_not_found(self, mock_chat_history_service):
        """
        Test getting non-existent session history
        """
        # Mock the service to return False for validation
        mock_chat_history_service.validate_session.return_value = False
        
        # Make the request
        response = client.get("/api/session/non-existent-session")
        
        # Assertions
        assert response.status_code == 404