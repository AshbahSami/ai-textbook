---
id: 0001
title: Create User Authentication with Better-Auth Spec
stage: spec
date: 2025-12-10
surface: agent
model: gemini-pro
feature: 1-user-auth-better-auth
branch: 1-user-auth-better-auth
user: unspecified
command: .specify/scripts/bash/create-phr.sh
labels: []
links:
  spec: specs/1-user-auth-better-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-user-auth-better-auth/spec.md
  - specs/1-user-auth-better-auth/checklists/requirements.md
tests:
  - None
---

## Prompt

/sp.specify Add Signup and Signin Functionality Using better-auth Objective: Implement a Signup and Signin flow using https://www.better-auth.com/ and integrate it with my existing Docusaurus frontend and Gemini CLI/Speckit+ backend workflows. Requirements: 1. Implement signup page and signin page inside the Docusaurus website (React-based). 2. Use better-auth to handle user creation, session management, password hashing, and token issuance. 3. At signup, ask structured questions about: - User’s software background (languages known, years of experience, tools used) - User’s hardware background (device type, CPU/GPU knowledge, OS used) - Preferred learning style (text, examples, code-heavy, step-by-step) 4. Store responses in user profile metadata (supported by better-auth). 5. Provide a secure API endpoint (Node/Express or Next.js server) that stores user metadata. 6. Integrate with the existing Gemini CLI/Speckit workflow so personalization rules can use the stored metadata. 7. Provide TypeScript interfaces for: - Signup Form Data - Signin Form Data - UserMetadata 8. Ensure forms have error handling, validation, and success redirects. 9. After signin, the user session should be accessible on all content pages so Gemini-powered explanations can be personalized. 10. Provide clear instructions on how to integrate Docusaurus React components with better-auth APIs. Success Criteria: - Fully working signup and signin components. - Metadata captured and retrievable via an authenticated session. - Gemini CLI workflows can access metadata to personalize content.

## Response snapshot

I have created a new feature branch `1-user-auth-better-auth` and a specification file `specs/1-user-auth-better-auth/spec.md` for the 'User Authentication with Better-Auth' feature. I have also created a checklist file `specs/1-user-auth-better-auth/checklists/requirements.md` and validated the specification.