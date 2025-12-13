import React from 'react';
import Layout from '@theme/Layout';

function Signin() {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <main>
        <h1>Sign In</h1>
        <form>
          <div>
            <label htmlFor="email">Email:</label>
            <input type="email" id="email" name="email" required />
          </div>
          <div>
            <label htmlFor="password">Password:</label>
            <input type="password" id="password" name="password" required />
          </div>
          <button type="submit">Sign In</button>
        </form>
      </main>
    </Layout>
  );
}

export default Signin;
