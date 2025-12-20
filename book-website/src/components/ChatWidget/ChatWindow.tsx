import React, { useState, useEffect, useRef } from 'react';
import Message from './Message';
import { chatAPI } from './api';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
}

interface ChatWindowProps {
  onClose: () => void;
  onUnreadMessage: () => void;
  initialHighlightedText?: string;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ onClose, onUnreadMessage, initialHighlightedText }) => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputText, setInputText] = useState(initialHighlightedText || '');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Initialize chat session
  useEffect(() => {
    const initSession = async () => {
      try {
        const sessionData = await chatAPI.createSession();
        setSessionId(sessionData.sessionId);
        
        // Add welcome message
        setMessages([{
          id: 'welcome',
          text: 'Hello! I\'m here to help you with documentation. What would you like to know?',
          sender: 'bot',
          timestamp: new Date()
        }]);
      } catch (error) {
        console.error('Error initializing chat session:', error);
      }
    };

    initSession();
  }, []);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async () => {
    if (!inputText.trim() || !sessionId || isLoading) return;

    // Add user message to UI immediately
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputText,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Get response from backend
      const response = await chatAPI.sendMessage(sessionId, inputText, initialHighlightedText);

      // Add bot response to UI
      const botMessage: Message = {
        id: `bot-${Date.now()}`,
        text: response.response,
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error: any) {
      console.error('Error sending message:', error);

      // Add error message to UI
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        text: error.message || 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chat-window">
      <div className="chat-header">
        <h3>Documentation Assistant</h3>
        <button className="close-button" onClick={onClose} aria-label="Close chat">
          ×
        </button>
      </div>
      
      <div className="chat-messages">
        {messages.map((message) => (
          <Message 
            key={message.id} 
            text={message.text} 
            sender={message.sender} 
            timestamp={message.timestamp} 
          />
        ))}
        {isLoading && (
          <Message 
            text="Thinking..." 
            sender="bot" 
            timestamp={new Date()} 
            isTyping={true} 
          />
        )}
        <div ref={messagesEndRef} />
      </div>
      
      <div className="chat-input-area">
        <textarea
          value={inputText}
          onChange={(e) => setInputText(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask about the documentation..."
          disabled={isLoading || !sessionId}
          rows={2}
        />
        <button 
          onClick={handleSendMessage} 
          disabled={!inputText.trim() || isLoading || !sessionId}
          className="send-button"
        >
          Send
        </button>
      </div>
    </div>
  );
};

export default ChatWindow;