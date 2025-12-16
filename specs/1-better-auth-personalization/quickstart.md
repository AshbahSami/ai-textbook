# Quickstart: Better-Auth & Personalization Integration

This guide provides a rapid introduction to integrating with the Better-Auth and Personalization API.

## 1. Better-Auth Setup

Before you begin, ensure you have configured your application with Better-Auth and obtained the necessary API credentials (Client ID, Client Secret, etc.). These will be used by your backend service to communicate with the Better-Auth API.

## 2. User Registration

To register a new user with personalization data, send a `POST` request to the `/api/auth/register` endpoint.

**Request Example (cURL):**

```bash
curl -X POST /api/auth/register \
     -H "Content-Type: application/json" \
     -d 
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

**Expected Response (201 Created):**

```json
{
  "message": "User registered successfully",
  "token": "optional_session_token_if_immediate_signin"
}
```

## 3. User Sign-in

To sign in an existing user, send a `POST` request to the `/api/auth/login` endpoint.

**Request Example (cURL):**

```bash
curl -X POST /api/auth/login \
     -H "Content-Type: application/json" \
     -d 
{
           "email": "user@example.com",
           "password": "SecurePassword123!"
         }
```

**Expected Response (200 OK):**

```json
{
  "message": "User authenticated successfully",
  "redirectUrl": "/personalized-dashboard"
}
```

_Note: A successful login will also set an `HTTP-only` session cookie._

<h2>4. User Logout</h2>

To log out the current user and invalidate their session, send a `POST` request to the `/api/auth/logout` endpoint.

**Request Example (cURL):**

```bash
curl -X POST /api/auth/logout \
     -H "Content-Type: application/json" \
     -H "Cookie: session_token=YOUR_SESSION_TOKEN"
```

**Expected Response (200 OK):**

```json
{
  "message": "User logged out successfully"
}
```
