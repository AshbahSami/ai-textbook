import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { auth, db } from '../../firebase'; // Corrected import path
import { createUserWithEmailAndPassword } from "firebase/auth";
import { doc, setDoc } from "firebase/firestore";

function Signup() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState('');
  const [hardwareBackground, setHardwareBackground] = useState('');
  const [error, setError] = useState('');

  const handleSignup = async (e) => {
    e.preventDefault();
    if (password !== confirmPassword) {
      setError("Passwords do not match");
      return;
    }
    try {
      const userCredential = await createUserWithEmailAndPassword(auth, email, password);
      const user = userCredential.user;
      await setDoc(doc(db, "users", user.uid), {
        email: user.email,
        softwareBackground: softwareBackground,
        hardwareBackground: hardwareBackground
      });
      // Redirect or show a success message
    } catch (error) {
      setError(error.message);
    }
  };

  return (
    <Layout title="Sign Up" description="Sign up for an account">
      <main>
        <h1>Sign Up</h1>
        {error && <p style={{ color: 'red' }}>{error}</p>}
        <form onSubmit={handleSignup}>
          <div>
            <label htmlFor="email">Email:</label>
            <input type="email" id="email" name="email" value={email} onChange={(e) => setEmail(e.target.value)} required />
          </div>
          <div>
            <label htmlFor="password">Password:</label>
            <input type="password" id="password" name="password" value={password} onChange={(e) => setPassword(e.target.value)} required />
          </div>
          <div>
            <label htmlFor="confirm-password">Confirm Password:</label>
            <input type="password" id="confirm-password" name="confirm-password" value={confirmPassword} onChange={(e) => setConfirmPassword(e.target.value)} required />
          </div>
          <div>
            <label htmlFor="software-background">Software Background:</label>
            <textarea id="software-background" name="software-background" value={softwareBackground} onChange={(e) => setSoftwareBackground(e.target.value)} />
          </div>
          <div>
            <label htmlFor="hardware-background">Hardware Background:</label>
            <textarea id="hardware-background" name="hardware-background" value={hardwareBackground} onChange={(e) => setHardwareBackground(e.target.value)} />
          </div>
          <button type="submit">Sign Up</button>
        </form>
      </main>
    </Layout>
  );
}

export default Signup;
