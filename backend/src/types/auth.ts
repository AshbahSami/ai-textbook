// backend/src/types/auth.ts

export interface User {
  email: string;
  betterAuthUserId: string;
}

export interface SoftwareBackground {
  primaryLanguages: string[];
  cloudFamiliarity: "None" | "Beginner" | "Intermediate" | "Advanced";
  preferredOs: "Linux" | "macOS" | "Windows" | "Hybrid";
  databaseExperience: string[];
}

export interface HardwareBackground {
  embeddedSystemsExperience: "Yes" | "No" | "Some C/Assembly";
  gpuFamiliarity: "None" | "Basic" | "Advanced";
}

export interface PersonalizationData {
  betterAuthUserId: string;
  softwareBackground: SoftwareBackground;
  hardwareBackground: HardwareBackground;
}
