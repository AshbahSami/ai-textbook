// temp-docusaurus-site/src/services/AuthAPI.ts
import axios from 'axios';
import { SoftwareBackground, HardwareBackground } from '../types/auth';



interface SignupRequest {
  email: string;
  password: string;
  background_data: {
    softwareBackground: SoftwareBackground;
    hardwareBackground: HardwareBackground;
  };
}

interface SignupResponse {
  message: string;
  userId: string;
}

interface LoginRequest {
  email: string;
  password: string;
}

interface LoginResponse {
  message: string;
  redirectUrl?: string;
  user?: { email: string; betterAuthUserId: string };
}

export class AuthAPI {
  private static instance: AuthAPI;
  private backendUrl: string;

  private constructor(backendUrl: string) {
    this.backendUrl = backendUrl;
  }

  public static getInstance(options?: { backendUrl: string }): AuthAPI {
    if (!AuthAPI.instance) {
      if (!options?.backendUrl) {
        throw new Error("AuthAPI requires a backendUrl to be provided during initialization.");
      }
      AuthAPI.instance = new AuthAPI(options.backendUrl);
    }
    return AuthAPI.instance;
  }

  public async register(data: SignupRequest): Promise<SignupResponse> {
    try {
      const response = await axios.post<SignupResponse>(`${this.backendUrl}/api/auth/register`, data);
      return response.data;
    } catch (error: any) {
      console.error("AuthAPI: Registration failed", error);
      throw new Error(error.response?.data?.message || error.message || "Registration failed");
    }
  }

  public async login(data: LoginRequest): Promise<LoginResponse> {
    try {
      const response = await axios.post<{ message: string; redirectUrl?: string; user?: { email: string; betterAuthUserId: string } }>(`${this.backendUrl}/api/auth/login`, data, {
        withCredentials: true, // Important for HTTP-only cookies
      });
      return response.data;
    } catch (error: any) {
      console.error("AuthAPI: Login failed", error);
      throw new Error(error.response?.data?.message || error.message || "Login failed");
    }
  }

  public async logout(): Promise<{ message: string }> {
    try {
      const response = await axios.post<{ message: string }>(`${this.backendUrl}/api/auth/logout`, {}, {
        withCredentials: true,
      });
      return response.data;
    } catch (error: any) {
      console.error("AuthAPI: Logout failed", error);
      throw new Error(error.response?.data?.message || error.message || "Logout failed");
    }
  }

  public async getAuthStatus(): Promise<{ isAuthenticated: boolean; user: { email: string; betterAuthUserId: string } | null }> {
    try {
      const response = await axios.get<{ isAuthenticated: boolean; user?: { email: string; betterAuthUserId: string } }>(`${this.backendUrl}/api/auth/status`, {
        withCredentials: true,
      });
      return {
        isAuthenticated: response.data.isAuthenticated,
        user: response.data.user || null,
      };
    } catch (error: any) {
      console.error("AuthAPI: Get auth status failed", error);
      return { isAuthenticated: false, user: null };
    }
  }

  public async getPersonalizationData(): Promise<any> { // This should return PersonalizationData type
    try {
      const response = await axios.get<any>(`${this.backendUrl}/api/user/personalization`, {
        withCredentials: true,
      });
      return response.data;
    } catch (error: any) {
      console.error("AuthAPI: Get personalization data failed", error);
      throw new Error(error.response?.data?.message || error.message || "Failed to fetch personalization data");
    }
  }
}
