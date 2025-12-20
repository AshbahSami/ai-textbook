// temp-docusaurus-site/src/pages/signup.tsx
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import axios from 'axios'; // Import axios
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { SoftwareBackground, HardwareBackground } from '../types/auth';

function Signup() {
  const { siteConfig } = useDocusaurusContext();
  const { BACKEND_API_URL } = siteConfig.customFields as { BACKEND_API_URL: string };

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState<SoftwareBackground>({
    primaryLanguages: [],
    cloudFamiliarity: "None",
    preferredOs: "Linux",
    databaseExperience: [],
  });
  const [hardwareBackground, setHardwareBackground] = useState<HardwareBackground>({
    embeddedSystemsExperience: "No",
    gpuFamiliarity: "None",
  });
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  const [passwordStrengthError, setPasswordStrengthError] = useState<string | null>(null);

  // Password strength validation function
  const validatePasswordStrength = (pwd: string) => {
    const minLength = 8;
    const hasUpperCase = /[A-Z]/.test(pwd);
    const hasLowerCase = /[a-z]/.test(pwd);
    const hasDigit = /[0-9]/.test(pwd);
    const hasSpecialChar = /[!@#$%^&*()_+\-=[\]{};':"\\|,.<>/?]/.test(pwd);

    if (pwd.length < minLength) {
      return `Password must be at least ${minLength} characters long.`;
    }
    if (!hasUpperCase) {
      return "Password must contain at least one uppercase letter.";
    }
    if (!hasLowerCase) {
      return "Password must contain at least one lowercase letter.";
    }
    if (!hasDigit) {
      return "Password must contain at least one number.";
    }
    if (!hasSpecialChar) {
      return "Password must contain at least one special character.";
    }
    return null; // Password is strong
  };

  const handlePasswordChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newPassword = e.target.value;
    setPassword(newPassword);
    const validationError = validatePasswordStrength(newPassword);
    setPasswordStrengthError(validationError);
  };

  const handleSoftwareChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const { name, value, selectedOptions } = e.target;
    if (e.target.multiple) {
      const selectedValues = Array.from(selectedOptions).map(option => option.value);
      setSoftwareBackground(prev => ({ ...prev, [name]: selectedValues }));
    } else {
      setSoftwareBackground(prev => ({ ...prev, [name]: value }));
    }
  };
  
  const handleHardwareChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const { name, value } = e.target;
    setHardwareBackground(prev => ({ ...prev, [name]: value }));
  };

  const handleSignup = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setSuccess(null);
    setPasswordStrengthError(null); // Clear previous password strength error

    if (password !== confirmPassword) {
      setError("Passwords do not match.");
      return;
    }

    const passwordValidationError = validatePasswordStrength(password);
    if (passwordValidationError) {
      setPasswordStrengthError(passwordValidationError);
      return;
    }

    try {
      const response = await axios.post(`${BACKEND_API_URL}/api/auth/register`, {
        email,
        password,
        background_data: {
          softwareBackground,
          hardwareBackground,
        },
      });

      if (response.status === 201) {
        setSuccess("Account created successfully! Redirecting to home page...");
        setTimeout(() => {
          window.location.href = '/'; // Redirect to home page
        }, 2000); // Redirect after 2 seconds
      }
    } catch (err: any) {
      console.error("Signup error:", err);
      let errorMessage = "Failed to create account. Please try again.";
      if (err.response?.data?.message) {
        errorMessage = err.response.data.message;
      } else if (err.message) {
        errorMessage = err.message;
      }
      setError(errorMessage);
    }
  };

  return (
    <Layout title="Sign Up" description="Sign up for an account">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Sign Up</h1>
            {error && <div className="alert alert--danger">{error}</div>}
            {success && <div className="alert alert--success">{success}</div>}
            <form onSubmit={handleSignup}>
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
                  onChange={handlePasswordChange}
                  required
                />
                 {passwordStrengthError && <div className="alert alert--warning margin-top--sm">{passwordStrengthError}</div>}
              </div>
              <div className="margin-bottom--md">
                <label htmlFor="confirm-password">Confirm Password:</label>
                <input
                  type="password"
                  id="confirm-password"
                  name="confirm-password"
                  className="form-control"
                  value={confirmPassword}
                  onChange={(e) => setConfirmPassword(e.target.value)}
                  required
                />
              </div>

              <h2>Technical Background Survey</h2>

              {/* Software Background */}
              <div className="margin-bottom--md">
                <label htmlFor="primaryLanguages">Primary Programming Language(s):</label>
                <select
                  id="primaryLanguages"
                  name="primaryLanguages"
                  multiple
                  className="form-control"
                  value={softwareBackground.primaryLanguages}
                  onChange={handleSoftwareChange}
                  required
                >
                  <option value="Python">Python</option>
                  <option value="JavaScript/TypeScript">JavaScript/TypeScript</option>
                  <option value="Java">Java</option>
                  <option value="C++">C++</option>
                  <option value="Go">Go</option>
                  <option value="Other">Other</option>
                </select>
              </div>

              <div className="margin-bottom--md">
                <label htmlFor="cloudFamiliarity">Familiarity with Cloud Platforms:</label>
                <select
                  id="cloudFamiliarity"
                  name="cloudFamiliarity"
                  className="form-control"
                  value={softwareBackground.cloudFamiliarity}
                  onChange={handleSoftwareChange}
                  required
                >
                  <option value="None">None</option>
                  <option value="Beginner">Beginner (e.g., single EC2 instance)</option>
                  <option value="Intermediate">Intermediate (e.g., Serverless)</option>
                  <option value="Advanced">Advanced (e.g., Kubernetes/Multi-region)</option>
                </select>
              </div>

              <div className="margin-bottom--md">
                <label htmlFor="preferredOs">Preferred Operating System for Development:</label>
                <select
                  id="preferredOs"
                  name="preferredOs"
                  className="form-control"
                  value={softwareBackground.preferredOs}
                  onChange={handleSoftwareChange}
                  required
                >
                  <option value="Linux">Linux</option>
                  <option value="macOS">macOS</option>
                  <option value="Windows">Windows</option>
                  <option value="Hybrid">Hybrid</option>
                </select>
              </div>

              <div className="margin-bottom--md">
                <label htmlFor="databaseExperience">Experience with Databases:</label>
                <select
                  id="databaseExperience"
                  name="databaseExperience"
                  multiple
                  className="form-control"
                  value={softwareBackground.databaseExperience}
                  onChange={handleSoftwareChange}
                  required
                >
                  <option value="SQL">SQL</option>
                  <option value="NoSQL">NoSQL</option>
                  <option value="Graph Databases">Graph Databases</option>
                  <option value="Other">Other</option>
                </select>
              </div>

              {/* Hardware Background */}
              <div className="margin-bottom--md">
                <label htmlFor="embeddedSystemsExperience">Experience with low-level programming/Embedded Systems:</label>
                <select
                  id="embeddedSystemsExperience"
                  name="embeddedSystemsExperience"
                  className="form-control"
                  value={hardwareBackground.embeddedSystemsExperience}
                  onChange={handleHardwareChange}
                  required
                >
                  <option value="No">No</option>
                  <option value="Yes">Yes</option>
                  <option value="Some C/Assembly">Some C/Assembly</option>
                </select>
              </div>

              <div className="margin-bottom--md">
                <label htmlFor="gpuFamiliarity">Familiarity with parallel computing/GPUs:</label>
                <select
                  id="gpuFamiliarity"
                  name="gpuFamiliarity"
                  className="form-control"
                  value={hardwareBackground.gpuFamiliarity}
                  onChange={handleHardwareChange}
                  required
                >
                  <option value="None">None</option>
                  <option value="Basic">Basic (e.g., using libraries)</option>
                  <option value="Advanced">Advanced (e.g., CUDA/OpenCL development)</option>
                </select>
              </div>

              <button type="submit" className="button button--primary button--block">Sign Up</button>
            </form>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Signup;
