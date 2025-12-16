# Data Model: Better-Auth & Personalization Integration

## Entities

### User

Represents an individual interacting with the system. Authentication details are primarily managed by Better-Auth.

**Attributes**:

*   `email`: (string, required, unique) - User's email address, used for login.
*   `betterAuthUserId`: (string, required, unique) - Unique identifier for the user provided by the Better-Auth service. This will be the primary link to personalization data.

**Relationships**:

*   One-to-one with `Personalization Data` via `betterAuthUserId`.

### Personalization Data

Stores a user's technical background information collected during signup for content personalization. This data is associated with the `User` entity through `betterAuthUserId` and stored within Better-Auth's user metadata.

**Attributes**:

*   `betterAuthUserId`: (string, required) - Foreign key linking to the `User` entity's `betterAuthUserId`.
*   `softwareBackground`: (JSON object, required) - Structured data capturing software-related background.
    *   `primaryLanguages`: (array of strings, e.g., "Python", "JavaScript/TypeScript")
    *   `cloudFamiliarity`: (string, e.g., "None", "Beginner", "Intermediate", "Advanced")
    *   `preferredOs`: (string, e.g., "Linux", "macOS", "Windows", "Hybrid")
    *   `databaseExperience`: (array of strings, e.g., "SQL", "NoSQL", "Graph Databases")
*   `hardwareBackground`: (JSON object, required) - Structured data capturing hardware-related background.
    *   `embeddedSystemsExperience`: (string, e.g., "Yes", "No", "Some C/Assembly")
    *   `gpuFamiliarity`: (string, e.g., "None", "Basic", "Advanced")

## Validation Rules (derived from Spec)

*   `email` must be a valid email format.
*   `email` must be unique across all registered users (handled by Better-Auth).
*   `betterAuthUserId` must be present for both `User` and `Personalization Data` entities.
*   `softwareBackground` and `hardwareBackground` fields must conform to the defined structures and acceptable values (e.g., specific options for multi-select/select fields).
