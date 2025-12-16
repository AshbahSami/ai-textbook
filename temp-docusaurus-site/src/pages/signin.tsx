// temp-docusaurus-site/src/pages/signin.tsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { AuthAPI } from '../services/AuthAPI'; // Import AuthAPI
import { Redirect } from '@docusaurus/router'; // For redirection
import { useAuth } from '../auth/AuthContext'; // Import useAuth hook

function Signin() {
  const { siteConfig } = useDocusaurusContext();
  const { BACKEND_API_URL } = siteConfig.customFields as { BACKEND_API_URL: string };

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [redirectTo, setRedirectTo] = useState<string | null>(null);

  const authAPI = AuthAPI.getInstance({ backendUrl: BACKEND_API_URL });
  const { login: authContextLogin } = useAuth(); // Get login function from AuthContext

  const handleSignin = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setSuccess(null);

    // Basic client-side validation
    if (!email || !password) {
      setError("Email and password are required.");
      return;
    }

    if (!/\S+@\S+\.\S+/.test(email)) {
      setError("Please enter a valid email address.");
      return;
    }

    if (password.length < 8) {
      setError("Password must be at least 8 characters long.");
      return;
    }

    try {
      const response = await authAPI.login({ email, password });

      if (response.message) {
        setSuccess(response.message);
        if (response.user) {
          authContextLogin(response.user.email, response.user.betterAuthUserId); // Update auth context with user details
        }
        if (response.redirectUrl) {
          setRedirectTo(response.redirectUrl);
        } else {
          setRedirectTo('/'); // Default redirect to home
        }
      }
    } catch (err: any) {
      console.error("Signin error:", err);
      let errorMessage = "Failed to sign in. Please check your credentials and try again.";
      if (err.message) {
        errorMessage = err.message;
      }
      setError(errorMessage);
    }
  };

  if (redirectTo) {
    return <Redirect to={redirectTo} />;
  }

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Sign In</h1>
            {error && <div className="alert alert--danger">{error}</div>}
            {success && <div className="alert alert--success">{success}</div>}
            <form onSubmit={handleSignin}>
              <div className="margin-bottom--md">
                <label htmlFor="email">Email:</label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  className="form-control"
                  value={email}
                  onChange={(e) => setEmail(e.target.value)}
                  required
                />
              </div>
              <div className="margin-bottom--md">
                <label htmlFor="password">Password:</label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  className="form-control"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  required
                />
              </div>

              <button type="submit" className="button button--primary button--block">Sign In</button>
            </form>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Signin;
