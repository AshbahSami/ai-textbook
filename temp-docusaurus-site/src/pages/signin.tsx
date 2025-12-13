import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { auth } from '../../firebase'; // Corrected import path
import { signInWithEmailAndPassword } from "firebase/auth";

function Signin() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');

  const handleSignin = async (e) => {
    e.preventDefault();
    try {
      await signInWithEmailAndPassword(auth, email, password);
      // Redirect or show a success message
    } catch (error) {
      setError(error.message);
    }
  };

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <main>
        <h1>Sign In</h1>
        {error && <p style={{ color: 'red' }}>{error}</p>}
        <form onSubmit={handleSignin}>
          <div>
            <label htmlFor="email">Email:</label>
            <input type="email" id="email" name="email" value={email} onChange={(e) => setEmail(e.target.value)} required />
          </div>
          <div>
            <label htmlFor="password">Password:</label>
            <input type="password" id="password" name="password" value={password} onChange={(e) => setPassword(e.target.value)} required />
          </div>
          <button type="submit">Sign In</button>
        </form>
      </main>
    </Layout>
  );
}

export default Signin;
