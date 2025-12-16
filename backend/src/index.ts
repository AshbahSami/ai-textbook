// backend/src/index.ts
import express from 'express';
import { json } from 'body-parser';
import cookieParser from 'cookie-parser'; // Import cookie-parser
import dotenv from 'dotenv';
import { BetterAuthService } from './services/BetterAuthService';
import { PersonalizationData } from './types/auth';

dotenv.config(); // Load environment variables from .env file

const app = express();
app.use(json()); // Enable JSON body parsing
app.use(cookieParser()); // Use cookie-parser middleware

const PORT = process.env.BACKEND_PORT || 3001;
const betterAuthService = BetterAuthService.getInstance({
  baseUrl: process.env.BETTER_AUTH_BASE_URL || 'https://www.better-auth.com',
});

// CORS for frontend
app.use((req, res, next) => {
  res.header('Access-Control-Allow-Origin', 'http://localhost:3000'); // Allow Docusaurus dev server
  res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');
  res.header('Access-Control-Allow-Credentials', 'true');
  next();
});

// Backend API Routes
app.post('/api/auth/register', async (req, res) => {
  const { email, password, background_data } = req.body;

  if (!email || !password || !background_data) {
    return res.status(400).send({ message: 'Missing required fields: email, password, background_data' });
  }

  try {
    // 1. Register with Better-Auth
    const registerResponse = await betterAuthService.register(email, password);
    const betterAuthUserId = registerResponse.userId;

    // 2. Store personalization data with Better-Auth (using userId)
    const personalizationData: PersonalizationData = {
      betterAuthUserId,
      softwareBackground: background_data.softwareBackground,
      hardwareBackground: background_data.hardwareBackground,
    };
    await betterAuthService.updatePersonalizationData(betterAuthUserId, personalizationData);

    // Frontend spec asks for optional token for immediate sign-in
    // If Better-Auth's register doesn't return a session token, we might need a separate login call here
    // For simplicity, assuming Better-Auth handles the session or returns enough for frontend to login
    res.status(201).send({ message: 'User registered successfully', userId: betterAuthUserId /*, token: registerResponse.token */ });
  } catch (error: any) {
    console.error('Backend /api/auth/register error:', error);
    // FR-011: Display user-friendly error message and suggest retry
    let errorMessage = 'Failed to register user. Please try again.';
    if (error.response?.data?.message) {
      errorMessage = error.response.data.message;
    } else if (error.message) {
      errorMessage = error.message;
    }
    res.status(500).send({ message: errorMessage });
  }
});

app.post('/api/auth/login', async (req, res) => {
  const { email, password } = req.body;

  if (!email || !password) {
    return res.status(400).send({ message: 'Missing required fields: email, password' });
  }

  try {
    const loginResponse = await betterAuthService.login(email, password);
    const { token, expiresIn } = loginResponse;

    // Set HTTP-only cookie
    res.cookie('session_token', token, {
      httpOnly: true,
      secure: process.env.NODE_ENV === 'production', // Use secure cookies in production
      maxAge: expiresIn * 1000, // maxAge is in milliseconds
      sameSite: 'lax', // Or 'strict' depending on requirements
    });

    res.status(200).send({ message: 'Login successful', redirectUrl: '/personalized-dashboard' });
  } catch (error: any) {
    console.error('Backend /api/auth/login error:', error);
    let errorMessage = 'Failed to log in. Please check your credentials and try again.';
    if (error.response?.status === 401) {
      errorMessage = 'Invalid credentials';
    } else if (error.response?.data?.message) {
      errorMessage = error.response.data.message;
    } else if (error.message) {
      errorMessage = error.message;
    }
    res.status(error.response?.status || 500).send({ message: errorMessage });
  }
});

app.post('/api/auth/logout', async (req, res) => {
  try {
    const sessionToken = req.cookies['session_token'];
    if (sessionToken) {
      await betterAuthService.logout(sessionToken); // Call Better-Auth logout
      res.clearCookie('session_token');
    }
    res.status(200).send({ message: 'Logged out successfully' });
  } catch (error: any) {
    console.error('Backend /api/auth/logout error:', error);
    res.status(500).send({ message: error.message || 'Failed to logout' });
  }
});

// Middleware to protect routes (example)
const isAuthenticated = async (req: express.Request, res: express.Response, next: express.NextFunction) => {
  const sessionToken = req.cookies['session_token'];
  if (sessionToken) {
    try {
      // In a real app, validate the token with Better-Auth to get user details
      // For now, mock validation and user ID extraction
      const validationResponse = await betterAuthService.validateToken(sessionToken); // Placeholder: assuming BetterAuthService has this method
      if (validationResponse && validationResponse.isValid) {
        res.locals.isAuthenticated = true;
        res.locals.user = { betterAuthUserId: validationResponse.userId, email: validationResponse.email }; // Store user info
        next();
      } else {
        res.locals.isAuthenticated = false;
        res.status(401).send({ message: 'Unauthorized: Invalid token' });
      }
    } catch (error) {
      console.error('Token validation failed:', error);
      res.locals.isAuthenticated = false;
      res.status(401).send({ message: 'Unauthorized: Token validation error' });
    }
  } else {
    res.locals.isAuthenticated = false;
    res.status(401).send({ message: 'Unauthorized: No token provided' });
  }
};

app.get('/api/auth/status', isAuthenticated, (req, res) => {
  res.status(200).send({ isAuthenticated: res.locals.isAuthenticated, user: res.locals.user, message: 'Authenticated' });
});

app.get('/api/user/personalization', isAuthenticated, async (req, res) => {
  try {
    const betterAuthUserId = res.locals.user.betterAuthUserId;
    const personalizationData = await betterAuthService.getPersonalizationData(betterAuthUserId);
    res.status(200).send(personalizationData);
  } catch (error: any) {
    console.error('Backend /api/user/personalization error:', error);
    res.status(500).send({ message: error.message || 'Failed to retrieve personalization data' });
  }
});

app.listen(PORT, () => {
  console.log(`Backend service running on http://localhost:${PORT}`);
});
