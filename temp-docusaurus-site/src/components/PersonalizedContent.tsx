// temp-docusaurus-site/src/components/PersonalizedContent.tsx
import React, { useEffect, useState } from 'react';
import { useAuth } from '../auth/AuthContext';
import { PersonalizationService } from '../services/PersonalizationService'; // Import PersonalizationService
import { PersonalizationData } from '../types/auth'; // Ensure this type is correctly defined in auth.ts
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const PersonalizedContent: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();
  const { BACKEND_API_URL } = siteConfig.customFields as { BACKEND_API_URL: string };
  const personalizationService = PersonalizationService.getInstance({ backendUrl: BACKEND_API_URL });

  const { isAuthenticated, user, loading, logout } = useAuth(); // Import logout function
  const [personalizationData, setPersonalizationData] = useState<PersonalizationData | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchPersonalizationData = async () => {
      if (!isAuthenticated || !user?.betterAuthUserId) {
        setPersonalizationData(null);
        return;
      }
      setError(null);
      try {
        const data = await personalizationService.getPersonalizationData(user.betterAuthUserId);
        setPersonalizationData(data);
      } catch (err: any) {
        console.error('Failed to fetch personalization data:', err);
        setError(err.message || 'Failed to load personalized content.');
        setPersonalizationData(null);
      }
    };

    if (!loading) {
      fetchPersonalizationData();
    }
  }, [isAuthenticated, user, loading]);

  if (loading) {
    return <div>Loading personalized content...</div>;
  }

  if (error) {
    return <div className="alert alert--danger">Error: {error}</div>;
  }

  if (!isAuthenticated) {
    return <div className="alert alert--info">Please log in to see personalized content.</div>;
  }

  if (!personalizationData) {
    return <div className="alert alert--info">No personalization data found or still loading.</div>;
  }

  return (
    <div className="card">
      <div className="card__header">
        <h3>Your Personalized Profile</h3>
      </div>
      <div className="card__body">
        <p><strong>Email:</strong> {user?.email}</p>
        <h4>Software Background</h4>
        <ul>
          <li>Primary Languages: {personalizationData.softwareBackground.primaryLanguages.join(', ')}</li>
          <li>Cloud Familiarity: {personalizationData.softwareBackground.cloudFamiliarity}</li>
          <li>Preferred OS: {personalizationData.softwareBackground.preferredOs}</li>
          <li>Database Experience: {personalizationData.softwareBackground.databaseExperience.join(', ')}</li>
        </ul>
        <h4>Hardware Background</h4>
        <ul>
          <li>Embedded Systems Experience: {personalizationData.hardwareBackground.embeddedSystemsExperience}</li>
          <li>GPU Familiarity: {personalizationData.hardwareBackground.gpuFamiliarity}</li>
        </ul>
        <button onClick={logout} className="button button--danger margin-top--md">
          Sign Out
        </button>
      </div>
    </div>
  );
};

export default PersonalizedContent;
