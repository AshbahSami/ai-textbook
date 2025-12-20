// API client for chat communication
// Using window object to access environment variable set by Docusaurus
declare global {
  interface Window {
    BACKEND_API_URL?: string;
  }
}

// Get the API base URL from either the global window variable (set by Docusaurus) or default
const API_BASE_URL = typeof window !== 'undefined' && window.BACKEND_API_URL
  ? `${window.BACKEND_API_URL}/api/v1`
  : 'http://localhost:8000/api/v1';

interface SessionData {
  sessionId: string;
  createdAt: string;
}

interface MessageResponse {
  response: string;
  sources: string[];
  sessionId: string;
}

class ChatAPI {
  async createSession(): Promise<SessionData> {
    try {
      const response = await fetch(`${API_BASE_URL}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`Failed to create session: ${response.status}`);
      }

      const data = await response.json();
      return {
        sessionId: data.session_id,
        createdAt: data.created_at,
      };
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }

  async sendMessage(sessionId: string, message: string, highlightedText?: string): Promise<MessageResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/${sessionId}`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          highlighted_text: highlightedText || null,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `Failed to send message: ${response.status}`);
      }

      return await response.json();
    } catch (error: any) {
      console.error('Error sending message:', error);
      if (error.message.includes('Failed to send message')) {
        throw new Error('Unable to connect to the chat service. Please check your connection and try again.');
      }
      throw error;
    }
  }

  async getSessionHistory(sessionId: string): Promise<any[]> {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/${sessionId}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error(`Failed to get session history: ${response.status}`);
      }

      const data = await response.json();
      return data.messages || [];
    } catch (error) {
      console.error('Error getting session history:', error);
      throw error;
    }
  }

  async queryDocumentation(query: string, highlightedText?: string): Promise<MessageResponse> {
    try {
      const response = await fetch(`${API_BASE_URL}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          highlighted_text: highlightedText || null,
        }),
      });

      if (!response.ok) {
        throw new Error(`Failed to query documentation: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error querying documentation:', error);
      throw error;
    }
  }
}

export const chatAPI = new ChatAPI();
export default ChatAPI;