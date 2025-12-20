// This file injects the config into the window object for use in the browser
// It reads from the Docusaurus config customFields

// Create a global config object with the environment variables
window.config = {
  BACKEND_API_URL: 'http://localhost:8000', // This will be overridden by Docusaurus config
};

// Override with values from Docusaurus config if available
if (typeof window !== 'undefined' && window.DOCUSAURUS_CONFIG) {
  window.config = {
    ...window.config,
    ...window.DOCUSAURUS_CONFIG.customFields
  };
}