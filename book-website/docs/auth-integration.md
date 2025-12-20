--- 
sidebar_position: 5
title: Authentication Integration
---

# Authentication Integration

This document describes the API endpoints and usage for the Better-Auth & Personalization feature, encompassing user registration, login, and personalization data retrieval.

## Overview

The authentication system leverages a separate backend service that interacts with the Better-Auth platform for core authentication (user management, session tokens). Frontend applications communicate with this backend service.

## API Endpoints

All authentication-related API endpoints are prefixed with `/api/auth`. User-specific data retrieval is under `/api/user`.

### 1. Register a New User

`POST /api/auth/register`

Registers a new user with Better-Auth and stores their technical background personalization data.

#### Request Body (application/json)

```json
{
  "email": "string (email format)",
  "password": "string",
  "background_data": {
    "softwareBackground": {
      "primaryLanguages": ["string"],
      "cloudFamiliarity": "string (enum: None, Beginner, Intermediate, Advanced)",
      "preferredOs": "string (enum: Linux, macOS, Windows, Hybrid)",
      "databaseExperience": ["string"]
    },
    "hardwareBackground": {
      "embeddedSystemsExperience": "string (enum: Yes, No, Some C/Assembly)",
      "gpuFamiliarity": "string (enum: None, Basic, Advanced)"
    }
  }
}
```

#### Responses

*   **201 Created**: User registered successfully.
    ```json
    {
      "message": "User registered successfully",
      "userId": "string"
    }
    ```
*   **400 Bad Request**: Missing required fields or invalid input.
*   **500 Internal Server Error**: Backend or Better-Auth service error.

#### Example (cURL)

```bash
curl -X POST /api/auth/register \
     -H "Content-Type: application/json" \
     -d 
```json
{
          "email": "user@example.com",
          "password": "SecurePassword123!",
          "background_data": {
            "softwareBackground": {
              "primaryLanguages": ["Python", "JavaScript/TypeScript"],
              "cloudFamiliarity": "Intermediate",
              "preferredOs": "Linux",
              "databaseExperience": ["SQL", "NoSQL"]
            },
            "hardwareBackground": {
              "embeddedSystemsExperience": "No",
              "gpuFamiliarity": "Basic"
            }
          }
        }
```
```

### 2. Authenticate User (Login)

`POST /api/auth/login`

Authenticates an existing user and establishes a session. Sets an HTTP-only cookie.

#### Request Body (application/json)

```json
{
  "email": "string (email format)",
  "password": "string"
}
```

#### Responses

*   **200 OK**: User authenticated successfully.
    *   **Headers**: `Set-Cookie: session_token=...; HttpOnly; Secure; Max-Age=...`
    ```json
    {
      "message": "Login successful",
      "redirectUrl": "string (optional URL for redirection)"
    }
    ```
*   **401 Unauthorized**: Invalid credentials.
*   **500 Internal Server Error**: Backend or Better-Auth service error.

#### Example (cURL)

```bash
curl -X POST /api/auth/login \
     -H "Content-Type: application/json" \
     -d 
```json
{
          "email": "user@example.com",
          "password": "SecurePassword123!"
        }
```
```

### 3. Retrieve Personalized Data

`GET /api/user/personalization`

Retrieves the authenticated user's technical background personalization data. Requires an active session.

#### Authentication

Requires an active session cookie (`session_token`).

#### Responses

*   **200 OK**: Personalization data retrieved successfully.
    ```json
    {
      "betterAuthUserId": "string",
      "softwareBackground": {
        "primaryLanguages": ["string"],
        "cloudFamiliarity": "string",
        "preferredOs": "string",
        "databaseExperience": ["string"]
      },
      "hardwareBackground": {
        "embeddedSystemsExperience": "string",
        "gpuFamiliarity": "string"
      }
    }
    ```
*   **401 Unauthorized**: No active session or invalid token.
*   **500 Internal Server Error**: Backend service error.

#### Example (cURL)

```bash
curl -X GET /api/user/personalization \
     -H "Cookie: session_token=YOUR_SESSION_TOKEN"
```

### 4. Logout User

`POST /api/auth/logout`

Invalidates the current user session and clears the session cookie.

#### Responses

*   **200 OK**: User logged out successfully.
    *   **Headers**: `Set-Cookie: session_token=; Expires=Thu, 01 Jan 1970 ...` (clears cookie)
    ```json
    {
      "message": "Logged out successfully"
    }
    ```
*   **500 Internal Server Error**: Backend service error.

#### Example (cURL)

```bash
curl -X POST /api/auth/logout \
     -H "Cookie: session_token=YOUR_SESSION_TOKEN"
```
