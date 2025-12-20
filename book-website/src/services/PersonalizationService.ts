// temp-docusaurus-site/src/services/PersonalizationService.ts

import axios from 'axios';
import { SoftwareBackground, HardwareBackground } from '../types/auth';



interface PersonalizationData {
  softwareBackground: SoftwareBackground;
  hardwareBackground: HardwareBackground;
}

export class PersonalizationService {
  private static instance: PersonalizationService;
  private backendUrl: string;

  private constructor(backendUrl: string) {
    this.backendUrl = backendUrl;
  }

  public static getInstance(options?: { backendUrl: string }): PersonalizationService {
    if (!PersonalizationService.instance) {
      if (!options?.backendUrl) {
        throw new Error("PersonalizationService requires a backendUrl to be provided during initialization.");
      }
      PersonalizationService.instance = new PersonalizationService(options.backendUrl);
    }
    return PersonalizationService.instance;
  }

  public async getPersonalizationData(betterAuthUserId: string): Promise<PersonalizationData | null> {
    try {
      const response = await axios.get(`${this.backendUrl}/api/personalization/${betterAuthUserId}`);
      if (response.status === 200) {
        return response.data;
      }
      return null;
    } catch (error) {
      console.error('Error fetching personalization data:', error);
      return null;
    }
  }
}
