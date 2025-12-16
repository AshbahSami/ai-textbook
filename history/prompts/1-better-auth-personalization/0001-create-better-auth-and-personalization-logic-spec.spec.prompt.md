---
id: 0001
title: Create Better-Auth and Personalization Logic Spec
stage: spec
date: 2025-12-13
surface: agent
model: gemini-pro
feature: 1-better-auth-personalization
branch: 1-better-auth-personalization
user: unspecified
command: .specify/scripts/bash/create-phr.sh
labels: []
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-better-auth-personalization/spec.md
  - specs/1-better-auth-personalization/checklists/requirements.md
tests:
  - no-tests-needed
---

## Prompt

```text
Better-Auth & Personalization Logic ### 1. Signup Flow Implementation 1. **Form Data:** The frontend form at `/auth/signup` must collect: * `email` (string, required) * `password` (string, required) * `background_data` (JSON object, required, containing answers to Q1-Q6 from the `/sp.plan`). 2. **Backend Endpoint (`/api/auth/register`):** * **Step A: Better-Auth Registration:** Call the Better-Auth API to create the user with the email and password. * Request body to Better-Auth: `{"email": <email>, "password": <password>}` * **Step B: Store Personalization Data:** After successful Better-Auth registration, store the `background_data` JSON. This can be done either: * **Option 1 (Preferred):** Using Better-Auth's User Metadata/Profile endpoint, if available. * **Option 2:** Storing it in the Daucasaurus internal User Profile database, linked by the Better-Auth User ID. * **Step C: Response:** Return a 201 Created status, optionally including the session token received from a subsequent Better-Auth login call for an immediate sign-in. ### 2. Signin Flow Implementation 1. **Form Data:** The frontend form at `/auth/signin` must collect: * `email` (string, required) * `password` (string, required) 2. **Backend Endpoint (`/api/auth/login`):** * **Step A: Better-Auth Login:** Call the Better-Auth session endpoint. * Request body to Better-Auth: `{"email": <email>, "password": <password>}` * **Step B: Token Handling:** Upon receiving a successful response (e.g., status 200) from Better-Auth, extract the session **Token** and its **Expiration** time. * **Step C: Session Creation:** Securely set the Token as an HTTP-only cookie or return it in the response body for client-side storage (use client-side storage only if necessary, HTTP-only cookie is preferred for security). * **Step D: Response:** Return a 200 OK status with a confirmation message or redirect the user to the personalized dashboard. ### 3. Personalization Data Retrieval Example When a user is logged in, use the Better-Auth User ID to fetch the personalization data (Q1-Q6) to dynamically modify the AI-book content: ```pseudocode FUNCTION personalize_content(user_id): // 1. Fetch personalization profile profile = database.fetch_profile_by_auth_id(user_id) // 2. Adjust AI-book prompt based on profile if "Python" in profile.primary_languages: ai_prompt_suffix = "Use Python examples and assume familiarity with pip/venv." else if "C#" in profile.primary_languages: ai_prompt_suffix = "Use C# examples and focus on .NET environment setup." // 3. Return the refined prompt for content generation return original_prompt + " " + ai_prompt_suffix"
```

## Response snapshot

```text
The feature specification for "Better-Auth & Personalization Logic" has been successfully generated and validated. The feature is ready for the next phase, which could be clarification or planning.
```
