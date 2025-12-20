// temp-docusaurus-site/src/components/PersonalizedContent.tsx
import React, { useState, useEffect } from 'react';
import { PersonalizationData } from '../types/auth';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Safe wrapper component that handles auth context errors gracefully
const PersonalizedContent: React.FC = () => {
  const [authState, setAuthState] = useState({
    isAuthenticated: false,
    user: null,
    loading: false,
    error: null
  });

  const { siteConfig } = useDocusaurusContext();
  const { BACKEND_API_URL } = siteConfig.customFields as { BACKEND_API_URL: string };

  // Attempt to access the auth context safely
  useEffect(() => {
    // Dynamically import the auth hook to handle errors gracefully
    const initAuth = async () => {
      try {
        const { useAuth } = await import('../auth/AuthContext');
        // This would work if AuthContext is properly provided
        // But we need to handle the error case
      } catch (error) {
        console.warn('Auth context not available, running in limited mode');
        setAuthState({
          isAuthenticated: false,
          user: null,
          loading: false,
          error: 'Auth context not available'
        });
      }
    };

    initAuth();
  }, []);

  // For now, just show a placeholder since auth context is not available
  return (
    <div className="card">
      <div className="card__header">
        <h3>Personalized Content</h3>
      </div>
      <div className="card__body">
        <p className="alert alert--info">
          Personalization requires authentication.
        </p>
      </div>
    </div>
  );
};

export default PersonalizedContent;
