import React from 'react';
import OriginalLayout from '@theme-original/Layout';

// This is the main layout component
export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      {/* The ChatWidget is now in Root.tsx, so no need to add it here */}
    </>
  );
}