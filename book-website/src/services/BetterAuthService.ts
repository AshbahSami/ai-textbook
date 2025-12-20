// temp-docusaurus-site/src/services/BetterAuthService.ts
import axios from 'axios';


const MAX_RETRIES = 2;
const RETRY_DELAY_MS = 1000; // 1 second

interface BetterAuthRegisterRequest {
  email: string;
  password: string;
}

interface BetterAuthRegisterResponse {
  userId: string;
  // ... other Better-Auth specific response data
}

interface BetterAuthLoginRequest {
  email: string;
  password: string;
}

interface BetterAuthLoginResponse {
  token: string;
  expiresIn: number;
  // ... other Better-Auth specific response data
}

interface BetterAuthPersonalizationDataUpdate {
  userId: string;
  metadata: any; // The personalization data
}

export class BetterAuthService {
  private static instance: BetterAuthService;
  private baseUrl: string;

  private constructor(baseUrl: string) {
    this.baseUrl = baseUrl;
  }

  public static getInstance(baseUrl?: string): BetterAuthService {
    if (!BetterAuthService.instance) {
      if (!baseUrl) {
        throw new Error("BetterAuthService requires a baseUrl to be provided during initialization.");
      }
      BetterAuthService.instance = new BetterAuthService(baseUrl);
    }
    return BetterAuthService.instance;
  }

  private async makeRequest<T>(
    method: 'post' | 'get' | 'put' | 'delete',
    url: string,
    data?: any,
    retries = MAX_RETRIES
  ): Promise<T> {
    try {
      const response = await axios({
        method,
        url: `${this.baseUrl}${url}`,
        data,
        withCredentials: true, // Important for cookies
      });
      return response.data;
    } catch (error: any) {
      if (retries > 0 && error.response?.status && [408, 429, 500, 502, 503, 504].includes(error.response.status)) {
        console.warn(`BetterAuthService: Retrying request to ${url}. Attempts left: ${retries}`);
        await new Promise(resolve => setTimeout(resolve, RETRY_DELAY_MS));
        return this.makeRequest<T>(method, url, data, retries - 1);
      }
      throw new Error(`BetterAuthService: API request failed - ${error.message || 'Unknown error'}`);
    }
  }

  public async register(email: string, password: string): Promise<BetterAuthRegisterResponse> {
    const response = await this.makeRequest<BetterAuthRegisterResponse>('post', '/api/v1/register', { email, password });
    return response;
  }

  public async login(email: string, password: string): Promise<BetterAuthLoginResponse> {
    const response = await this.makeRequest<BetterAuthLoginResponse>('post', '/api/v1/login', { email, password });
    return response;
  }

  public async logout(token: string): Promise<any> {
    // Better-Auth might have a specific logout endpoint or rely on cookie invalidation
    // This is a placeholder for how you might call Better-Auth's logout
    const response = await this.makeRequest<any>('post', '/api/v1/logout', { token });
    return response;
  }

  public async updatePersonalizationData(userId: string, data: any): Promise<any> {
    // Assuming Better-Auth has a metadata/profile update endpoint
    const response = await this.makeRequest<any>('put', `/api/v1/users/${userId}/metadata`, { metadata: data });
    return response;
  }

  public async getPersonalizationData(userId: string): Promise<any> {
    const response = await this.makeRequest<any>('get', `/api/v1/users/${userId}/metadata`);
    return response;
  }
}
