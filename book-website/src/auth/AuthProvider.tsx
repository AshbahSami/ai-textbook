// temp-docusaurus-site/src/auth/AuthProvider.tsx
import React, { useState, useEffect, useCallback } from 'react';
import { AuthContext } from './AuthContext';
import { AuthAPI } from '../services/AuthAPI'; // Assuming AuthAPI is in ../services/
import { User } from '../types/auth'; // Import User interface
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface AuthProviderProps {
  children: React.ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const { siteConfig } = useDocusaurusContext();
  const { BACKEND_API_URL } = siteConfig.customFields as { BACKEND_API_URL: string };
  const authAPI = AuthAPI.getInstance({ backendUrl: BACKEND_API_URL });

  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState<boolean>(true);

  const checkAuthStatus = useCallback(async () => {
    try {
      setLoading(true);
      const response = await authAPI.getAuthStatus(); // Implement this in AuthAPI
      if (response.isAuthenticated && response.user) {
        setIsAuthenticated(true);
        setUser(response.user);
      } else {
        setIsAuthenticated(false);
        setUser(null);
      }
    } catch (error) {
      console.error('Error checking auth status:', error);
      setIsAuthenticated(false);
      setUser(null);
    } finally {
      setLoading(false);
    }
  }, [authAPI]);

  useEffect(() => {
    checkAuthStatus();
  }, [checkAuthStatus, authAPI]);

  const login = useCallback((email: string, betterAuthUserId: string) => {
    setIsAuthenticated(true);
    setUser({ email, betterAuthUserId });
  }, []);

  const logout = useCallback(async () => {
    try {
      await authAPI.logout();
      setIsAuthenticated(false);
      setUser(null);
    } catch (error) {
      console.error('Error during logout:', error);
      // Optionally, force logout even if API call fails
      setIsAuthenticated(false);
      setUser(null);
    }
  }, [authAPI]);

  return (
    <AuthContext.Provider value={{ isAuthenticated, user, login, logout, loading }}>
      {children}
    </AuthContext.Provider>
  );
};
