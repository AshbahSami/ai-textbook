// ChatWidgetInjector.js
// This component injects the ChatWidget into the Root of the Docusaurus app

import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget/ChatWidget';

const ChatWidgetInjector = () => {
  // Only render the chat widget if we're in the browser
  if (typeof window !== 'undefined') {
    return <ChatWidget />;
  }

  return null;
};

export default ChatWidgetInjector;