import pytest
from unittest.mock import Mock, patch, MagicMock
from sqlalchemy import text
from src.services.chat_history_service import ChatHistoryService


class TestChatHistoryService:
    """
    Unit tests for ChatHistoryService
    """
    
    @pytest.fixture
    def chat_history_service(self):
        with patch('src.services.chat_history_service.SessionLocal') as mock_session_local:
            # Mock the database session
            mock_session = Mock()
            mock_session_local.return_value.__enter__.return_value = mock_session
            
            service = ChatHistoryService()
            service.db_session = mock_session
            
            return service, mock_session
    
    def test_save_interaction(self, chat_history_service):
        """
        Test saving an interaction to the database
        """
        service, mock_session = chat_history_service
        
        # Mock the database execute result
        mock_result = Mock()
        mock_result.fetchone.return_value = [1]  # Return an ID
        mock_session.execute.return_value = mock_result
        
        # Call the method under test
        interaction_id = service.save_interaction(
            session_id="test-session",
            user_query="test query",
            context_used="test context",
            llm_response="test response",
            source_url="https://example.com"
        )
        
        # Assertions
        assert interaction_id == 1
        mock_session.execute.assert_called_once()
        mock_session.commit.assert_called_once()
    
    def test_get_session_history(self, chat_history_service):
        """
        Test retrieving session history
        """
        service, mock_session = chat_history_service
        
        # Mock the database result
        mock_row1 = (1, "user query 1", "response 1", "https://example.com", "2023-01-01")
        mock_row2 = (2, "user query 2", "response 2", "https://example.com", "2023-01-02")
        mock_result = Mock()
        mock_result.fetchall.return_value = [mock_row1, mock_row2]
        mock_session.execute.return_value = mock_result
        
        # Call the method under test
        history = service.get_session_history("test-session")
        
        # Assertions
        assert len(history) == 2
        assert history[0]['user_query'] == "user query 1"
        assert history[1]['user_query'] == "user query 2"
        mock_session.execute.assert_called_once()
    
    def test_get_all_sessions(self, chat_history_service):
        """
        Test retrieving all sessions
        """
        service, mock_session = chat_history_service
        
        # Mock the database result
        mock_result = Mock()
        mock_result.fetchall.return_value = [("session1",), ("session2",)]
        mock_session.execute.return_value = mock_result
        
        # Call the method under test
        sessions = service.get_all_sessions()
        
        # Assertions
        assert len(sessions) == 2
        assert "session1" in sessions
        assert "session2" in sessions
        mock_session.execute.assert_called_once()
    
    def test_validate_session(self, chat_history_service):
        """
        Test validating a session
        """
        service, mock_session = chat_history_service
        
        # Mock the database result for valid session
        mock_result_valid = Mock()
        mock_result_valid.fetchone.return_value = [1]  # One interaction exists
        mock_session.execute.return_value = mock_result_valid
        
        # Call the method under test
        is_valid = service.validate_session("valid-session")
        
        # Assertions
        assert is_valid is True
        mock_session.execute.assert_called_once()
    
    def test_validate_session_invalid(self, chat_history_service):
        """
        Test validating an invalid session
        """
        service, mock_session = chat_history_service
        
        # Mock the database result for invalid session
        mock_result_invalid = Mock()
        mock_result_invalid.fetchone.return_value = [0]  # No interactions exist
        mock_session.execute.return_value = mock_result_invalid
        
        # Call the method under test
        is_invalid = not service.validate_session("invalid-session")
        
        # Assertions
        assert is_invalid is True
        mock_session.execute.assert_called_once()