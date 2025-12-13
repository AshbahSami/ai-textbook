import React from 'react';
import { AuthProvider } from '../auth/AuthProvider';

function Root({children}) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}

export default Root;