---
id: 0002
title: Add Signup + Signin Using better-auth
stage: plan
date: 2025-12-10
surface: agent
model: gemini-pro
feature: 1-user-auth-better-auth
branch: 1-user-auth-better-auth
user: ashbahsami60
command: /sp.plan Add Signup + Signin Using better-auth
labels: []
links:
  spec: specs/1-user-auth-better-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-user-auth-better-auth/plan.md
  - specs/1-user-auth-better-auth/research.md
  - specs/1-user-auth-better-auth/data-model.md
  - specs/1-user-auth-better-auth/quickstart.md
  - specs/1-user-auth-better-auth/contracts/openapi.yml
tests: []
---

## Prompt

/sp.plan Add Signup + Signin Using better-auth

1. Initialize better-auth
   - Install SDK
   - Configure auth provider
   - Create server endpoints for signup/signin and metadata storage

2. Create User Metadata Schema
   - softwareBackground: string[]
   - hardwareBackground: string[]
   - experienceYears: number
   - preferredLearningStyle: string

3. Build Signup Page in Docusaurus
   - Form fields: email, password, confirm password
   - Background questions
   - POST to /api/auth/signup

4. Build Signin Page
   - Form fields: email, password
   - POST to /api/auth/signin
   - Redirect to dashboard

5. Integrate Session Management
   - Use better-auth client in Docusaurus
   - Store JWT/session token
   - Add global context provider

6. Connect Personalization
   - Expose GET /api/user/profile
   - Gemini CLI scripts use this metadata for personalized responses

7. Testing & Validation
   - Form validation
   - Session persistence
   - Metadata accuracy

## Response snapshot

Generated the implementation plan, research document, data model, API contracts (OpenAPI), and quickstart guide for the "User Authentication with Better-Auth" feature. These artifacts define the technical context, constitutional compliance, and initial design for the feature, including integration with Docusaurus and better-auth.
