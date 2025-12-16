
import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';
import './styles.css';

const CHATBOT_CONTAINER_ID = 'rag-chatbot-container';

const Chatbot = ({ selectedText }) => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');

  useEffect(() => {
    if (selectedText) {
      setMessages([{ text: `Explain: "${selectedText}"`, sender: 'user' }]);
      handleSendMessage(selectedText);
    }
  }, [selectedText]);

  const handleInputChange = (event) => {
    setInputValue(event.target.value);
  };

  const handleSendMessage = async (text = inputValue) => {
    if (text.trim() === '') return;

    const newMessages = [...messages, { text, sender: 'user' }];
    setMessages(newMessages);
    setInputValue('');

    try {
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: 'user123', // Replace with a dynamic user ID if needed
          message: text,
        }),
      });

      if (!response.ok) {
        throw new Error('Network response was not ok');
      }

      const data = await response.json();
      const botMessage = data.response; // Make sure this matches your RAG chatbot's response format

      setMessages([...newMessages, { text: botMessage, sender: 'bot' }]);
    } catch (error) {
      console.error('Error fetching from chatbot:', error);
      setMessages([
        ...newMessages,
        { text: 'Sorry, I am having trouble connecting. Please try again later.', sender: 'bot' },
      ]);
    }
  };

  const closeChatbot = () => {
    const container = document.getElementById(CHATBOT_CONTAINER_ID);
    if (container) {
      ReactDOM.unmountComponentAtNode(container);
    }
  };

  return (
    <div className="chatbot-container open">
      <div className="chatbot-header">
        <h2>RAG Chatbot</h2>
        <button className="close-btn" onClick={closeChatbot}>×</button>
      </div>
      <div className="chatbot-body">
        <div className="chatbot-messages">
          {messages.map((message, index) => (
            <div key={index} className={`message ${message.sender}`}>
              {message.text}
            </div>
          ))}
        </div>
        <div className="chatbot-input">
          <input
            type="text"
            value={inputValue}
            onChange={handleInputChange}
            onKeyPress={(event) => event.key === 'Enter' && handleSendMessage()}
            placeholder="Ask me anything..."
          />
          <button onClick={() => handleSendMessage()}>Send</button>
        </div>
      </div>
    </div>
  );
};

export default Chatbot;
