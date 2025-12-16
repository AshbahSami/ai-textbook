// backend/src/services/__tests__/BetterAuthService.test.ts
import axios from 'axios';
import { BetterAuthService } from '../BetterAuthService';

// Mock axios
jest.mock('axios');
const mockedAxios = axios as jest.Mocked<typeof axios>;

// Mock process.env for testing
const mockEnv = {
  BETTER_AUTH_BASE_URL: 'https://mock.better-auth.com',
};

// Temporarily set process.env for testing
const originalEnv = process.env;
process.env = { ...originalEnv, ...mockEnv };

describe('BetterAuthService', () => {
  let betterAuthService: BetterAuthService;

  beforeEach(() => {
    // Reset the singleton instance for each test to ensure isolation
    (BetterAuthService as any).instance = undefined;
    betterAuthService = BetterAuthService.getInstance();
    jest.clearAllMocks();
  });

  afterAll(() => {
    process.env = originalEnv; // Restore original environment variables
  });

  it('should be a singleton instance', () => {
    const instance1 = BetterAuthService.getInstance();
    const instance2 = BetterAuthService.getInstance();
    expect(instance1).toBe(instance2);
  });

  describe('register', () => {
    it('should successfully register a user', async () => {
      const mockResponse = { userId: 'betterAuthUserId123' };
      mockedAxios.post.mockResolvedValueOnce({ data: mockResponse });

      const email = 'test@example.com';
      const password = 'password123';
      await expect(betterAuthService.register(email, password)).resolves.toEqual(mockResponse);
      expect(mockedAxios.post).toHaveBeenCalledWith(
        `${mockEnv.BETTER_AUTH_BASE_URL}/api/v1/register`,
        { email, password }
      );
    });

    it('should retry on specific 5xx errors', async () => {
      mockedAxios.post
        .mockRejectedValueOnce({ response: { status: 500, data: 'Server Error' } })
        .mockResolvedValueOnce({ data: { userId: 'retriedUserId' } });

      const email = 'test@example.com';
      const password = 'password123';
      await expect(betterAuthService.register(email, password)).resolves.toEqual({ userId: 'retriedUserId' });
      expect(mockedAxios.post).toHaveBeenCalledTimes(2);
    });

    it('should throw error after max retries', async () => {
      mockedAxios.post
        .mockRejectedValueOnce({ response: { status: 503, data: 'Service Unavailable' } })
        .mockRejectedValueOnce({ response: { status: 503, data: 'Service Unavailable' } })
        .mockRejectedValueOnce({ response: { status: 503, data: 'Service Unavailable' } });

      const email = 'test@example.com';
      const password = 'password123';
      await expect(betterAuthService.register(email, password)).rejects.toThrow('BetterAuthService: API request failed');
      expect(mockedAxios.post).toHaveBeenCalledTimes(3); // 1 initial + 2 retries
    });
  });

  describe('login', () => {
    it('should successfully log in a user', async () => {
      const mockResponse = { token: 'sessionToken123', expiresIn: 3600 };
      mockedAxios.post.mockResolvedValueOnce({ data: mockResponse });

      const email = 'test@example.com';
      const password = 'password123';
      await expect(betterAuthService.login(email, password)).resolves.toEqual(mockResponse);
      expect(mockedAxios.post).toHaveBeenCalledWith(
        `${mockEnv.BETTER_AUTH_BASE_URL}/api/v1/login`,
        { email, password }
      );
    });
  });

  describe('logout', () => {
    it('should successfully log out a user', async () => {
      const mockResponse = { message: 'Logged out' };
      mockedAxios.post.mockResolvedValueOnce({ data: mockResponse });

      const token = 'sessionToken123';
      await expect(betterAuthService.logout(token)).resolves.toEqual(mockResponse);
      expect(mockedAxios.post).toHaveBeenCalledWith(
        `${mockEnv.BETTER_AUTH_BASE_URL}/api/v1/logout`,
        { token }
      );
    });
  });

  describe('updatePersonalizationData', () => {
    it('should successfully update personalization data', async () => {
      const mockResponse = { success: true };
      mockedAxios.put.mockResolvedValueOnce({ data: mockResponse });

      const userId = 'userId123';
      const data = { software: 'JavaScript' };
      await expect(betterAuthService.updatePersonalizationData(userId, data)).resolves.toEqual(mockResponse);
      expect(mockedAxios.put).toHaveBeenCalledWith(
        `${mockEnv.BETTER_AUTH_BASE_URL}/api/v1/users/${userId}/metadata`,
        { metadata: data }
      );
    });
  });

  describe('getPersonalizationData', () => {
    it('should successfully retrieve personalization data', async () => {
      const mockResponse = { software: 'JavaScript' };
      mockedAxios.get.mockResolvedValueOnce({ data: mockResponse });

      const userId = 'userId123';
      await expect(betterAuthService.getPersonalizationData(userId)).resolves.toEqual(mockResponse);
      expect(mockedAxios.get).toHaveBeenCalledWith(
        `${mockEnv.BETTER_AUTH_BASE_URL}/api/v1/users/${userId}/metadata`
      );
    });
  });

  describe('validateToken', () => {
    it('should return isValid true for a non-empty token', async () => {
      const token = 'validToken';
      const result = await betterAuthService.validateToken(token);
      expect(result.isValid).toBe(true);
      expect(result.userId).toBeDefined();
      expect(result.email).toBeDefined();
    });

    it('should return isValid false for an empty token', async () => {
      const token = '';
      const result = await betterAuthService.validateToken(token);
      expect(result.isValid).toBe(false);
    });
  });
});
