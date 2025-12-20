// temp-docusaurus-site/src/services/__tests__/AuthAPI.test.ts
import axios from 'axios';
import { AuthAPI } from '../AuthAPI';

// Mock axios
jest.mock('axios');
const mockedAxios = axios as jest.Mocked<typeof axios>;

describe('AuthAPI', () => {
  let authAPI: AuthAPI;
  const BACKEND_API_URL = 'http://localhost:3001';

  beforeEach(() => {
    // Reset the singleton instance for each test to ensure isolation
    (AuthAPI as any).instance = undefined; 
    authAPI = AuthAPI.getInstance();
    // Set process.env.BACKEND_API_URL for tests
    process.env.BACKEND_API_URL = BACKEND_API_URL;
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  it('should be a singleton instance', () => {
    const instance1 = AuthAPI.getInstance();
    const instance2 = AuthAPI.getInstance();
    expect(instance1).toBe(instance2);
  });

  describe('register', () => {
    it('should successfully register a user', async () => {
      const mockResponse = { message: 'User registered successfully', userId: 'testUserId' };
      mockedAxios.post.mockResolvedValueOnce({ data: mockResponse, status: 201 });

      const signupData = {
        email: 'test@example.com',
        password: 'password123',
        background_data: {
          softwareBackground: { primaryLanguages: ['Python'], cloudFamiliarity: 'None', preferredOs: 'Linux', databaseExperience: [] },
          hardwareBackground: { embeddedSystemsExperience: 'No', gpuFamiliarity: 'None' },
        },
      };

      await expect(authAPI.register(signupData)).resolves.toEqual(mockResponse);
      expect(mockedAxios.post).toHaveBeenCalledWith(
        `${BACKEND_API_URL}/api/auth/register`,
        signupData
      );
    });

    it('should throw an error if registration fails', async () => {
      const errorMessage = 'Registration failed';
      mockedAxios.post.mockRejectedValueOnce({ response: { data: { message: errorMessage } } });

      const signupData = {
        email: 'test@example.com',
        password: 'password123',
        background_data: {
          softwareBackground: { primaryLanguages: ['Python'], cloudFamiliarity: 'None', preferredOs: 'Linux', databaseExperience: [] },
          hardwareBackground: { embeddedSystemsExperience: 'No', gpuFamiliarity: 'None' },
        },
      };

      await expect(authAPI.register(signupData)).rejects.toThrow(errorMessage);
    });
  });

  describe('login', () => {
    it('should successfully log in a user', async () => {
      const mockResponse = { message: 'Login successful', user: { email: 'test@example.com', betterAuthUserId: 'testUserId' } };
      mockedAxios.post.mockResolvedValueOnce({ data: mockResponse, status: 200 });

      const loginData = { email: 'test@example.com', password: 'password123' };

      await expect(authAPI.login(loginData)).resolves.toEqual(mockResponse);
      expect(mockedAxios.post).toHaveBeenCalledWith(
        `${BACKEND_API_URL}/api/auth/login`,
        loginData,
        { withCredentials: true }
      );
    });

    it('should throw an error if login fails', async () => {
      const errorMessage = 'Invalid credentials';
      mockedAxios.post.mockRejectedValueOnce({ response: { data: { message: errorMessage } } });

      const loginData = { email: 'test@example.com', password: 'password123' };

      await expect(authAPI.login(loginData)).rejects.toThrow(errorMessage);
    });
  });

  describe('logout', () => {
    it('should successfully log out a user', async () => {
      const mockResponse = { message: 'Logged out successfully' };
      mockedAxios.post.mockResolvedValueOnce({ data: mockResponse, status: 200 });

      await expect(authAPI.logout()).resolves.toEqual(mockResponse);
      expect(mockedAxios.post).toHaveBeenCalledWith(
        `${BACKEND_API_URL}/api/auth/logout`,
        {},
        { withCredentials: true }
      );
    });

    it('should throw an error if logout fails', async () => {
      const errorMessage = 'Logout failed';
      mockedAxios.post.mockRejectedValueOnce({ response: { data: { message: errorMessage } } });

      await expect(authAPI.logout()).rejects.toThrow(errorMessage);
    });
  });

  describe('getAuthStatus', () => {
    it('should return authenticated status and user data if logged in', async () => {
      const mockResponse = { isAuthenticated: true, user: { email: 'test@example.com', betterAuthUserId: 'testUserId' } };
      mockedAxios.get.mockResolvedValueOnce({ data: mockResponse, status: 200 });

      await expect(authAPI.getAuthStatus()).resolves.toEqual(mockResponse);
      expect(mockedAxios.get).toHaveBeenCalledWith(
        `${BACKEND_API_URL}/api/auth/status`,
        { withCredentials: true }
      );
    });

    it('should return unauthenticated status if not logged in', async () => {
      const mockResponse = { isAuthenticated: false, user: null };
      mockedAxios.get.mockResolvedValueOnce({ data: mockResponse, status: 200 });

      await expect(authAPI.getAuthStatus()).resolves.toEqual(mockResponse);
    });

    it('should return unauthenticated status if fetching status fails', async () => {
      mockedAxios.get.mockRejectedValueOnce(new Error('Network error'));

      await expect(authAPI.getAuthStatus()).resolves.toEqual({ isAuthenticated: false, user: null });
    });
  });

  describe('getPersonalizationData', () => {
    it('should successfully fetch personalization data', async () => {
      const mockPersonalizationData = {
        softwareBackground: { primaryLanguages: ['Python'] },
        hardwareBackground: { gpuFamiliarity: 'Advanced' }
      };
      mockedAxios.get.mockResolvedValueOnce({ data: mockPersonalizationData, status: 200 });

      await expect(authAPI.getPersonalizationData()).resolves.toEqual(mockPersonalizationData);
      expect(mockedAxios.get).toHaveBeenCalledWith(
        `${BACKEND_API_URL}/api/user/personalization`,
        { withCredentials: true }
      );
    });

    it('should throw an error if fetching personalization data fails', async () => {
      const errorMessage = 'Failed to fetch data';
      mockedAxios.get.mockRejectedValueOnce({ response: { data: { message: errorMessage } } });

      await expect(authAPI.getPersonalizationData()).rejects.toThrow(errorMessage);
    });
  });
});
