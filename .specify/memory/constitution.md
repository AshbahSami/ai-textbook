<!--
Sync Impact Report:
- Version change: 1.2.0 → 1.3.0
- Modified principles:
  - 5. Authentication and Personalization Standard: Expanded with detailed goals, vision, requirements, constraints, and success criteria.
- Added sections:
  - None
- Removed sections:
  - None
- Templates requiring updates:
  - None (Checked `plan-template.md`, `spec-template.md`, `tasks-template.md`, command files, and runtime guidance docs - no direct impacts found needing updates yet due to this constitution change alone.)
- Follow-up TODOs:
  - None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles (The 'Why' for the Book)

### 1. AI-Native Documentation
The textbook's primary purpose is to serve as the **authoritative knowledge base** for the RAG Chatbot. Documentation must be generated and structured using AI-native tools (e.g., **Claude Code/Spec-Kit Plus**) to ensure efficiency and integration readiness.

### 2. Actionable Knowledge Base
The book must be optimized for machine readability and retrieval. Content must be clear, granular, and easily translatable into a structured database to maximize the performance of the integrated **RAG system**.

### 3. Comprehensive Coverage
The final textbook must provide a complete and holistic understanding of the entire system architecture, from the **ROS 2 Nervous System** up through the **VLA Cognitive Brain**.

---

## Key Standards (The 'How' for the Book)

### 1. Technical Accuracy Standard
All content—including code snippets, mathematical derivations, and technical specifications—must be rigorously checked for correctness and align with the latest versions of **ROS 2** and the **NVIDIA Isaac Platform**.

### 2. Modular Structure Standard
The textbook must be organized into four distinct, sequential modules as outlined in the curriculum, ensuring logical flow and ease of indexing for the **RAG Chatbot**.

### 3. Tool-Specific Format
The book's final output format and style must comply with the specifications and conventions enforced by the generative tool used (**Claude Code/Spec-Kit Plus**) to ensure compatibility and consistency.

### 4. Documentation Platform Standard
All final documentation output (the textbook chapters) must be rendered into Markdown files that strictly adhere to the file naming and front-matter conventions required for publishing on the Docusaurus documentation framework. This ensures the content is ready for immediate deployment as a web-based document.

### 5. Authentication and Personalization Standard

#### 🎯 5.1 Goal

To securely implement **User Signup** and **User Signin** functionality on the Daucasaurus website using the **Better-Auth** service, and to establish a mechanism during signup to collect user technical background (hardware and software) to enable future content personalization.

#### 🌟 5.2 Vision & Rationale

*   **Security:** Utilize Better-Auth's expertise to manage sensitive authentication processes, ensuring user data is secure and compliance is maintained.
*   **Personalization:** Create an informed user profile that allows the Daucasaurus AI-book engine to tailor content (examples, complexity, tool recommendations, and technical focus) to the user's existing knowledge base. For example, a user with a strong Python/Linux background will receive different content suggestions than a user with a C#/Windows background.
*   **User Experience (UX):** Ensure the authentication process is simple, fast, and seamlessly integrated into the Daucasaurus aesthetic.

#### 🛠️ 5.3 High-Level Requirements

| Feature | Description | Better-Auth Dependency |
| :--- | :--- | :--- |
| **User Signup** | Implement a form that registers a new user via Better-Auth. Must include a required section for background questions. | Yes (API integration) |
| **User Signin** | Implement a form that authenticates an existing user via Better-Auth. | Yes (API integration) |
| **Background Collection** | A set of questions asked *during* signup to categorize the user's technical profile (software/hardware). | No (Custom database field/metadata) |
| **Session Management** | Securely manage the user session state post-signin (e.g., tokens, cookies). | Yes (As per Better-Auth recommendations) |

#### 🛑 5.4 Constraints

*   **Use of Better-Auth:** All primary authentication logic (password hashing, token issuance, user storage) **must** utilize the Better-Auth service API (`https://www.better-auth.com/`).
*   **Minimal Data:** Only collect essential data for authentication (Email, Password) and personalization (Background Survey).
*   **Compliance:** Adhere to standard data privacy practices (GDPR, CCPA as applicable).

#### ✅ 5.5 Success Criteria

*   A new user can successfully register and login.
*   The user's technical background answers are stored and retrievable upon successful signup.
*   The Signin process is secured via HTTPS/WSS and correctly utilizes Better-Auth tokens/sessions.
*   A logged-in user can access a personalized dashboard area.

---

## Success Criteria (The 'What to Deliver')

### 1. Functional RAG Chatbot
A fully operational RAG Chatbot (**FastAPI, Agents/ChatKit**) that can accurately query and respond based on the content of the AI-native textbook.

### 2. VLA-Integrated Control
Successful demonstration of an integrated **Vision-Language-Action (VLA)** model performing cognitive planning and high-level control of a simulated humanoid system.

### 3. Complete Textbook
A comprehensive, four-module AI-native textbook covering: **ROS 2**, **Digital Twin (Simulation)**, **AI-Robot Brain (Isaac)**, and **VLA Integration**.

---

## Constraints (The 'Boundaries')

### 1. Tool Adherence
Advice is limited to and must utilize the specified tool stack: **ROS 2**, **NVIDIA Isaac Platform**, **Claude Code/Spec-Kit Plus**, and **OpenAI Agents/ChatKit SDKs**.

### 2. Scope Limitation
Guidance is strictly limited to the technical scope of the four course modules and the resulting humanoid robotics system. Avoid providing generic LLM or non-robotics advice.

---

## Governance

Amendments to this constitution require a documented proposal, review by the project advisors, and a clear migration plan for any affected components. All project artifacts and development activities MUST adhere to these principles and standards.

**Version**: 1.3.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-13