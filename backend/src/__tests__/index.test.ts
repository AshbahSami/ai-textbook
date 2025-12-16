// backend/src/__tests__/index.test.ts
import request from 'supertest';
import express from 'express';
import { json } from 'body-parser';
import cookieParser from 'cookie-parser';
import dotenv from 'dotenv';
import { BetterAuthService } from '../src/services/BetterAuthService';
import { PersonalizationData } from '../src/types/auth';

// Mock dotenv config
dotenv.config = jest.fn();

// Mock BetterAuthService
jest.mock('../src/services/BetterAuthService');
const mockBetterAuthService = BetterAuthService.getInstance() as jest.Mocked<BetterAuthService>;

let app: express.Application;

beforeAll(() => {
  // Dynamically import the app after mocks are set up
  app = express();
  app.use(json());
  app.use(cookieParser());
  
  // Mock CORS for testing
  app.use((req, res, next) => {
    res.header('Access-Control-Allow-Origin', 'http://localhost:3000');
    res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');
    res.header('Access-Control-Allow-Credentials', 'true');
    next();
  });

  // Re-import the actual routes after setting up middleware to ensure they use the mocked service
  // This is a common pattern when you need to inject mocks before routes are defined
  const { default: actualApp } = require('../src/index'); // Adjust path as needed
  app.use(actualApp); // Use the actual app's routes
});

describe('Backend API Routes', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    // Ensure the BetterAuthService mock methods are reset for each test
    mockBetterAuthService.register.mockReset();
    mockBetterAuthService.updatePersonalizationData.mockReset();
    mockBetterAuthService.login.mockReset();
    mockBetterAuthService.logout.mockReset();
    mockBetterAuthService.validateToken.mockReset();
    mockBetterAuthService.getPersonalizationData.mockReset();
  });

  describe('POST /api/auth/register', () => {
    it('should register a user and store personalization data', async () => {
      mockBetterAuthService.register.mockResolvedValue({ userId: 'newUserId123' });
      mockBetterAuthService.updatePersonalizationData.mockResolvedValue({});

      const signupData = {
        email: 'register@test.com',
        password: 'password123',
        background_data: {
          softwareBackground: { primaryLanguages: ['Python'], cloudFamiliarity: 'None', preferredOs: 'Linux', databaseExperience: [] },
          hardwareBackground: { embeddedSystemsExperience: 'No', gpuFamiliarity: 'None' },
        },
      };

      const res = await request(app)
        .post('/api/auth/register')
        .send(signupData);

      expect(res.statusCode).toEqual(201);
      expect(res.body).toEqual({ message: 'User registered successfully', userId: 'newUserId123' });
      expect(mockBetterAuthService.register).toHaveBeenCalledWith(signupData.email, signupData.password);
      expect(mockBetterAuthService.updatePersonalizationData).toHaveBeenCalledWith(
        'newUserId123',
        {
          betterAuthUserId: 'newUserId123',
          softwareBackground: signupData.background_data.softwareBackground,
          hardwareBackground: signupData.background_data.hardwareBackground,
        }
      );
    });

    it('should return 400 if required fields are missing', async () => {
      const res = await request(app)
        .post('/api/auth/register')
        .send({ email: 'test@test.com', password: 'password123' }); // Missing background_data

      expect(res.statusCode).toEqual(400);
      expect(res.body).toEqual({ message: 'Missing required fields: email, password, background_data' });
    });

    it('should handle errors from BetterAuthService during registration', async () => {
      mockBetterAuthService.register.mockRejectedValue(new Error('BetterAuth registration failed'));

      const signupData = {
        email: 'error@test.com',
        password: 'password123',
        background_data: {
          softwareBackground: { primaryLanguages: ['Python'], cloudFamiliarity: 'None', preferredOs: 'Linux', databaseExperience: [] },
          hardwareBackground: { embeddedSystemsExperience: 'No', gpuFamiliarity: 'None' },
        },
      };

      const res = await request(app)
        .post('/api/auth/register')
        .send(signupData);

      expect(res.statusCode).toEqual(500);
      expect(res.body).toEqual({ message: 'BetterAuth registration failed' });
    });
  });

  describe('POST /api/auth/login', () => {
    it('should log in a user and set a session cookie', async () => {
      mockBetterAuthService.login.mockResolvedValue({ token: 'mockToken', expiresIn: 3600 });

      const loginData = { email: 'login@test.com', password: 'password123' };

      const res = await request(app)
        .post('/api/auth/login')
        .send(loginData);

      expect(res.statusCode).toEqual(200);
      expect(res.body).toEqual({ message: 'Login successful', redirectUrl: '/personalized-dashboard' });
      expect(res.headers['set-cookie'][0]).toContain('session_token=mockToken');
      expect(mockBetterAuthService.login).toHaveBeenCalledWith(loginData.email, loginData.password);
    });

    it('should return 400 if required fields are missing', async () => {
      const res = await request(app)
        .post('/api/auth/login')
        .send({ email: 'login@test.com' }); // Missing password

      expect(res.statusCode).toEqual(400);
      expect(res.body).toEqual({ message: 'Missing required fields: email, password' });
    });

    it('should handle invalid credentials from BetterAuthService', async () => {
      mockBetterAuthService.login.mockRejectedValue({ response: { status: 401, data: { message: 'Invalid credentials' } } });

      const loginData = { email: 'wrong@test.com', password: 'badpassword' };

      const res = await request(app)
        .post('/api/auth/login')
        .send(loginData);

      expect(res.statusCode).toEqual(401);
      expect(res.body).toEqual({ message: 'Invalid credentials' });
    });
  });

  describe('POST /api/auth/logout', () => {
    it('should log out a user and clear the session cookie', async () => {
      mockBetterAuthService.logout.mockResolvedValue({});

      const res = await request(app)
        .post('/api/auth/logout')
        .set('Cookie', 'session_token=someToken');

      expect(res.statusCode).toEqual(200);
      expect(res.body).toEqual({ message: 'Logged out successfully' });
      expect(res.headers['set-cookie'][0]).toContain('session_token=;'); // Checks for cleared cookie
      expect(mockBetterAuthService.logout).toHaveBeenCalledWith('someToken');
    });

    it('should return 500 if BetterAuthService logout fails', async () => {
      mockBetterAuthService.logout.mockRejectedValue(new Error('BetterAuth logout failed'));

      const res = await request(app)
        .post('/api/auth/logout')
        .set('Cookie', 'session_token=someToken');

      expect(res.statusCode).toEqual(500);
      expect(res.body).toEqual({ message: 'BetterAuth logout failed' });
    });
  });

  describe('GET /api/auth/status', () => {
    it('should return authenticated status if session token is valid', async () => {
      mockBetterAuthService.validateToken.mockResolvedValue({ isValid: true, userId: 'validUser', email: 'user@test.com' });

      const res = await request(app)
        .get('/api/auth/status')
        .set('Cookie', 'session_token=validToken');

      expect(res.statusCode).toEqual(200);
      expect(res.body).toEqual({ isAuthenticated: true, user: { betterAuthUserId: 'validUser', email: 'user@test.com' }, message: 'Authenticated' });
      expect(mockBetterAuthService.validateToken).toHaveBeenCalledWith('validToken');
    });

    it('should return 401 if session token is invalid or missing', async () => {
      mockBetterAuthService.validateToken.mockResolvedValue({ isValid: false });

      const res = await request(app)
        .get('/api/auth/status')
        .set('Cookie', 'session_token=invalidToken');

      expect(res.statusCode).toEqual(401);
      expect(res.body).toEqual({ message: 'Unauthorized: Invalid token' });
    });

    it('should return 401 if no session token is provided', async () => {
      const res = await request(app)
        .get('/api/auth/status');

      expect(res.statusCode).toEqual(401);
      expect(res.body).toEqual({ message: 'Unauthorized: No token provided' });
    });
  });

  describe('GET /api/user/personalization', () => {
    it('should return personalization data for an authenticated user', async () => {
      mockBetterAuthService.validateToken.mockResolvedValue({ isValid: true, userId: 'validUser', email: 'user@test.com' });
      const mockPersonalizationData = { softwareBackground: { primaryLanguages: ['JS'] } };
      mockBetterAuthService.getPersonalizationData.mockResolvedValue(mockPersonalizationData);

      const res = await request(app)
        .get('/api/user/personalization')
        .set('Cookie', 'session_token=validToken');

      expect(res.statusCode).toEqual(200);
      expect(res.body).toEqual(mockPersonalizationData);
      expect(mockBetterAuthService.getPersonalizationData).toHaveBeenCalledWith('validUser');
    });

    it('should return 401 if user is not authenticated', async () => {
      mockBetterAuthService.validateToken.mockResolvedValue({ isValid: false });

      const res = await request(app)
        .get('/api/user/personalization')
        .set('Cookie', 'session_token=invalidToken');

      expect(res.statusCode).toEqual(401);
    });

    it('should return 500 if fetching personalization data fails', async () => {
      mockBetterAuthService.validateToken.mockResolvedValue({ isValid: true, userId: 'validUser', email: 'user@test.com' });
      mockBetterAuthService.getPersonalizationData.mockRejectedValue(new Error('DB error'));

      const res = await request(app)
        .get('/api/user/personalization')
        .set('Cookie', 'session_token=validToken');

      expect(res.statusCode).toEqual(500);
      expect(res.body).toEqual({ message: 'DB error' });
    });
  });
});
