import React from 'react';
import Layout from '@theme/Layout';

function Signup() {
  return (
    <Layout title="Sign Up" description="Sign up for an account">
      <main>
        <h1>Sign Up</h1>
        <form>
          <div>
            <label htmlFor="email">Email:</label>
            <input type="email" id="email" name="email" required />
          </div>
          <div>
            <label htmlFor="password">Password:</label>
            <input type="password" id="password" name="password" required />
          </div>
          <div>
            <label htmlFor="confirm-password">Confirm Password:</label>
            <input type="password" id="confirm-password" name="confirm-password" required />
          </div>
          <button type="submit">Sign Up</button>
        </form>
      </main>
    </Layout>
  );
}

export default Signup;
