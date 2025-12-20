import requests
import os

def test_api():
    """
    Simple test to verify the API is working
    """
    # Assuming the API is running on the default port
    base_url = os.getenv("BACKEND_API_URL", "http://localhost:8000")
    
    try:
        # Test the root endpoint
        response = requests.get(f"{base_url}/")
        assert response.status_code == 200
        print("✓ API root endpoint is working")
        
        # Test the health endpoint
        response = requests.get(f"{base_url}/health")
        assert response.status_code == 200
        print("✓ Health endpoint is working")
        
        # Test creating a session
        response = requests.post(f"{base_url}/api/chat/session")
        assert response.status_code == 200
        session_data = response.json()
        session_id = session_data.get("session_id")
        assert session_id is not None
        print("✓ Session creation is working")
        
        print(f"All tests passed! Sample session ID: {session_id}")
        
    except Exception as e:
        print(f"Test failed with error: {e}")
        return False
    
    return True

if __name__ == "__main__":
    test_api()