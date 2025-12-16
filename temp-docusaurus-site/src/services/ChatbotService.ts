// temp-docusaurus-site/src/services/ChatbotService.ts

import axios from 'axios';
import { v4 as uuidv4 } from 'uuid';

interface ContextualQuery {
  session_id: string;
  selected_text: string;
  full_page_url: string;
  surrounding_context: string;
}

interface ExplanationResponse {
  explanation: string;
  source_url: string;
  session_id: string;
  supplemental_context_used: boolean;
}

export class ChatbotService {
  private static instance: ChatbotService;
  private backendUrl: string;
  private sessionId: string;

  private constructor(backendUrl: string) {
    this.backendUrl = backendUrl;
    this.sessionId = uuidv4();
  }

  public static getInstance(options?: { backendUrl: string }): ChatbotService {
    if (!ChatbotService.instance) {
      if (!options?.backendUrl) {
        throw new Error("ChatbotService requires a backendUrl to be provided during initialization.");
      }
      ChatbotService.instance = new ChatbotService(options.backendUrl);
    }
    return ChatbotService.instance;
  }

  public async getContextualExplanation(selectedText: string, surroundingContext: string): Promise<ExplanationResponse | null> {
    try {
      const query: ContextualQuery = {
        session_id: this.sessionId,
        selected_text: selectedText,
        full_page_url: window.location.href,
        surrounding_context: surroundingContext,
      };
      const response = await axios.post(`${this.backendUrl}/api/context-query`, query);
      if (response.status === 200) {
        return response.data;
      }
      return null;
    } catch (error) {
      console.error('Error fetching contextual explanation:', error);
      return null;
    }
  }
}
