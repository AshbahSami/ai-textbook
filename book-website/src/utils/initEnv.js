window.ENV = {
  SUPABASE_URL: window.APP_CONFIG?.SUPABASE_URL || 'https://your-project.supabase.co',
  SUPABASE_PUBLISHABLE_KEY: window.APP_CONFIG?.SUPABASE_PUBLISHABLE_KEY || 'your-anon-key',
};