import React from 'react';
import {AuthProvider} from './auth/AuthProvider';

export function wrapClientApp(App) {
  return function AppWithProviders(props) {
    return (
      <AuthProvider>
        <App {...props} />
      </AuthProvider>
    );
  };
}