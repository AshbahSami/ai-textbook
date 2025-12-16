// temp-docusaurus-site/src/components/Chatbot/ChatbotWindow.tsx
import React, { useState, useEffect } from 'react';
import './Chatbot.css';
import { ChatbotService } from '../../services/ChatbotService';

interface ChatbotWindowProps {
  onClose: () => void;
}

const ChatbotWindow: React.FC<ChatbotWindowProps> = ({ onClose }) => {
  const [messages, setMessages] = useState<{ text: string; sender: 'user' | 'bot' }[]>([]);
  const [inputValue, setInputValue] = useState('');
  const chatbotService = ChatbotService.getInstance({ backendUrl: 'http://localhost:8000' });

  useEffect(() => {
    setMessages([{ text: 'Hello! How can I help you today?', sender: 'bot' }]);
  }, []);

  const handleSendMessage = async () => {
    if (inputValue.trim() === '') return;

    const newMessages = [...messages, { text: inputValue, sender: 'user' as 'user' }];
    setMessages(newMessages);
    setInputValue('');

    const response = await chatbotService.getContextualExplanation(inputValue, '');
    if (response) {
      setMessages([...newMessages, { text: response.explanation, sender: 'bot' as 'bot' }]);
    } else {
      setMessages([...newMessages, { text: 'Sorry, I could not get an explanation.', sender: 'bot' as 'bot' }]);
    }
  };

  return (
    <div className="chatbot-window">
      <div className="chatbot-header">
        <span>Chatbot</span>
        <button className="chatbot-close-button" onClick={onClose}>
          &times;
        </button>
      </div>
      <div className="chatbot-messages">
        {messages.map((msg, index) => (
          <div key={index} className={`message ${'msg.sender'}`}>
            {msg.text}
          </div>
        ))}
      </div>
      <div className="chatbot-input-container">
        <input
          type="text"
          className="chatbot-input"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={(e) => e.key === 'Enter' && handleSendMessage()}
        />
        <button className="chatbot-send-button" onClick={handleSendMessage}>
          &#9658;
        </button>
      </div>
    </div>
  );
};

export default ChatbotWindow;
