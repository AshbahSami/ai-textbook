// temp-docusaurus-site/src/auth/AuthContext.ts
import { createContext, useContext } from 'react';

interface AuthContextType {
  isAuthenticated: boolean;
  user: { email: string; betterAuthUserId: string } | null; // Basic user info
  login: (email: string, betterAuthUserId: string) => void;
  logout: () => void;
  loading: boolean;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
