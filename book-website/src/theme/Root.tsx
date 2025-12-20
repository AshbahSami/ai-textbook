import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

// This is the root component that wraps the entire app
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}