# Feature Specification: Better-Auth & Personalization Logic

**Feature Branch**: `1-better-auth-personalization`  
**Created**: 2025-12-13  
**Status**: Draft  
**Input**: User description: "Better-Auth & Personalization Logic ### 1. Signup Flow Implementation 1. **Form Data:** The frontend form at `/auth/signup` must collect: * `email` (string, required) * `password` (string, required) * `background_data` (JSON object, required, containing answers to Q1-Q6 from the `/sp.plan`). 2. **Backend Endpoint (`/api/auth/register`):** * **Step A: Better-Auth Registration:** Call the Better-Auth API to create the user with the email and password. * Request body to Better-Auth: `{"email": <email>, "password": <password>}` * **Step B: Store Personalization Data:** After successful Better-Auth registration, store the `background_data` JSON. This can be done either: * **Option 1 (Preferred):** Using Better-Auth's User Metadata/Profile endpoint, if available. * **Option 2:** Storing it in the Daucasaurus internal User Profile database, linked by the Better-Auth User ID. * **Step C: Response:** Return a 201 Created status, optionally including the session token received from a subsequent Better-Auth login call for an immediate sign-in. ### 2. Signin Flow Implementation 1. **Form Data:** The frontend form at `/auth/signin` must collect: * `email` (string, required) * `password` (string, required) 2. **Backend Endpoint (`/api/auth/login`):** * **Step A: Better-Auth Login:** Call the Better-Auth session endpoint. * Request body to Better-Auth: `{"email": <email>, "password": <password>}` * **Step B: Token Handling:** Upon receiving a successful response (e.g., status 200) from Better-Auth, extract the session **Token** and its **Expiration** time. * **Step C: Session Creation:** Securely set the Token as an HTTP-only cookie or return it in the response body for client-side storage (use client-side storage only if necessary, HTTP-only cookie is preferred for security). * **Step D: Response:** Return a 200 OK status with a confirmation message or redirect the user to the personalized dashboard. ### 3. Personalization Data Retrieval Example When a user is logged in, use the Better-Auth User ID to fetch the personalization data (Q1-Q6) to dynamically modify the AI-book content: ```pseudocode FUNCTION personalize_content(user_id): // 1. Fetch personalization profile profile = database.fetch_profile_by_auth_id(user_id) // 2. Adjust AI-book prompt based on profile if "Python" in profile.primary_languages: ai_prompt_suffix = "Use Python examples and assume familiarity with pip/venv." else if "C#" in profile.primary_languages: ai_prompt_suffix = "Use C# examples and focus on .NET environment setup." // 3. Return the refined prompt for content generation return original_prompt + " " + ai_prompt_suffix"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Signup (Priority: P1)

As a new user, I want to create an account so I can access personalized content.

**Why this priority**: Essential for user acquisition and enabling personalization features.

**Independent Test**: A user can navigate to the signup page, fill in all required information including background data, successfully register, and receive confirmation of account creation.

**Acceptance Scenarios**:

1.  **Given** I am on the `/auth/signup` page, **When** I provide a unique email, a password, and my technical background data, **Then** my account is created with Better-Auth and my background data is stored.
2.  **Given** I am on the `/auth/signup` page, **When** I provide an already registered email, **Then** I receive an error message indicating the email is taken.
3.  **Given** I am on the `/auth/signup` page, **When** I provide invalid background data, **Then** I receive an error message and my account is not created.
4.  **Given** I am on the `/auth/signup` page, **When** Better-Auth is unavailable or returns an error, **Then** I receive a user-friendly error message, suggested to retry, and the system attempts basic retries.

### User Story 2 - Existing User Sign-in (Priority: P1)

As an existing user, I want to sign in to my account so I can access personalized content.

**Why this priority**: Essential for existing user engagement and access to features.

**Independent Test**: A user can navigate to the sign-in page, enter their registered credentials, successfully sign in, and be redirected to a personalized dashboard.

**Acceptance Scenarios**:

1.  **Given** I am on the `/auth/signin` page, **When** I provide my registered email and correct password, **Then** I am successfully authenticated by Better-Auth and redirected to a personalized dashboard.
2.  **Given** I am on the `/auth/signin` page, **When** I provide an unregistered email or incorrect password, **Then** I receive an error message indicating invalid credentials.
3.  **Given** I am logged in, **When** I close and reopen the browser, **Then** my session is maintained (if using HTTP-only cookie).
4.  **Given** I am on the `/auth/signin` page, **When** Better-Auth is unavailable or returns an error, **Then** I receive a user-friendly error message, suggested to retry, and the system attempts basic retries.

### User Story 3 - Personalized Content Experience (Priority: P2)

As a logged-in user, I want the AI-book content to be tailored to my technical background so it is more relevant to me.

**Why this priority**: Enhances user experience and value proposition.

**Independent Test**: A logged-in user with specific background data (e.g., Python experience) views content that demonstrates Python-specific examples.

**Acceptance Scenarios**:

1.  **Given** I am logged in, **When** I view AI-book content, **Then** the content dynamically adjusts based on my stored technical background.
2.  **Given** I am logged in with a Python-focused background, **When** I view a code example, **Then** the example code is presented in Python.

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST provide a user registration interface at `/auth/signup` that collects email, password, and structured technical background data.
-   **FR-002**: The system MUST, upon registration, call the Better-Auth API to create a new user with the provided email and password.
-   **FR-003**: The system MUST, after successful Better-Auth registration, store the collected technical background data using Better-Auth's User Metadata/Profile endpoint, linked by the Better-Auth User ID.
-   **FR-004**: The system MUST return a `201 Created` status upon successful user registration, optionally including a session token for immediate sign-in.
-   **FR-005**: The system MUST provide a user login interface at `/auth/signin` that collects email and password.
-   **FR-006**: The system MUST, upon login, call the Better-Auth API session endpoint with the provided email and password.
-   **FR-007**: The system MUST, upon successful Better-Auth login, securely create a user session by setting an HTTP-only cookie containing the token and its expiration time provided by Better-Auth.
-   **FR-008**: The system MUST return a `200 OK` status upon successful user login, or redirect the user to a personalized dashboard.
-   **FR-009**: The system MUST, for logged-in users, retrieve their stored personalization data using their Better-Auth User ID.
-   **FR-010**: The system MUST use the retrieved personalization data to dynamically modify and tailor the AI-book content presented to the user.
-   **FR-011**: The system MUST, upon encountering an error or unavailability of the Better-Auth service during registration or login, display a user-friendly error message, suggest a retry, and implement basic retry logic (e.g., 1-2 immediate retries).

### Key Entities *(include if feature involves data)*

-   **User**: Represents an individual interacting with the system, identified by an email, password, and a Better-Auth User ID.
-   **Personalization Data**: Structured information about a user's technical background (e.g., software skills, hardware experience) stored via Better-Auth's User Metadata/Profile endpoint and used to customize content delivery.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: 100% of new users can successfully register and log in to the platform.
-   **SC-002**: Personalization data is accurately captured and stored for 100% of registered users.
-   **SC-003**: The sign-in process securely authenticates users and establishes a session in under 2 seconds.
-   **SC-004**: AI-book content dynamically adapts based on user personalization data for all logged-in users.

## Clarifications

### Session 2025-12-13
- Q: Which option should be definitively chosen for storing the user's `background_data`? → A: Use Better-Auth's User Metadata/Profile endpoint.
- Q: What will be the primary mechanism for managing user sessions after successful login? → A: HTTP-only cookie
- Q: How should the system behave if the Better-Auth service is unavailable or returns an error during user registration or login? → A: Display user-friendly error message and suggest retry