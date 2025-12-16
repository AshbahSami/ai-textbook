// temp-docusaurus-site/src/plugins/chatbot-plugin/index.js
const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'chatbot-plugin',
    getRoot: ({ App }) => {
      return function Root({ children }) {
        const ChatbotButton = require('@site/src/components/Chatbot/ChatbotButton').default;
        return (
          <>
            <App>{children}</App>
            <ChatbotButton />
          </>
        );
      };
    },
  };
};
