import React from 'react';

interface MessageProps {
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
  isTyping?: boolean;
}

const Message: React.FC<MessageProps> = ({ text, sender, timestamp, isTyping = false }) => {
  // Format timestamp
  const timeString = timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  return (
    <div className={`message ${sender}`}>
      <div className="message-content">
        {isTyping ? (
          <div className="typing-indicator">
            <div className="typing-dot"></div>
            <div className="typing-dot"></div>
            <div className="typing-dot"></div>
          </div>
        ) : (
          <p>{text}</p>
        )}
        <div className="message-timestamp">{timeString}</div>
      </div>
    </div>
  );
};

export default Message;