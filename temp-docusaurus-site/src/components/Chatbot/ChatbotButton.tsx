// temp-docusaurus-site/src/components/Chatbot/ChatbotButton.tsx
import React, { useState } from 'react';
import './Chatbot.css';
import ChatbotWindow from './ChatbotWindow';

const ChatbotButton: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChatWindow = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      <button className="chatbot-button" onClick={toggleChatWindow}>
        <span>🤖</span>
      </button>
      {isOpen && <ChatbotWindow onClose={toggleChatWindow} />}
    </>
  );
};

export default ChatbotButton;
