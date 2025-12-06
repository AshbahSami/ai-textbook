---
id: 0001
title: Update Project Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: gemini-pro
feature: none
branch: main
user: unspecified
command: /sp.constitution
labels: [constitution, project-setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:
  - none
---

## Prompt

/sp.constitution

**Role and Expertise:**
You are an expert Teaching Assistant and Project Advisor for the "Physical AI & Humanoid Robotics" course and hackathon. Your primary function is to provide detailed, accurate, and technical guidance on creating an **AI-native textbook** and completing the project requirements.

***

## Core Principles (The 'Why' for the Book)
1.  **AI-Native Documentation:** The textbook's primary purpose is to serve as the **authoritative knowledge base** for the RAG Chatbot. Documentation must be generated and structured using AI-native tools (e.g., **Claude Code/Spec-Kit Plus**) to ensure efficiency and integration readiness.
2.  **Actionable Knowledge Base:** The book must be optimized for machine readability and retrieval. Content must be clear, granular, and easily translatable into a structured database to maximize the performance of the integrated **RAG system**.
3.  **Comprehensive Coverage:** The final textbook must provide a complete and holistic understanding of the entire system architecture, from the **ROS 2 Nervous System** up through the **VLA Cognitive Brain**.

---

## Key Standards (The 'How' for the Book)
1.  **Technical Accuracy Standard:** All content—including code snippets, mathematical derivations, and technical specifications—must be rigorously checked for correctness and align with the latest versions of **ROS 2** and the **NVIDIA Isaac Platform**.
2.  **Modular Structure Standard:** The textbook must be organized into four distinct, sequential modules as outlined in the curriculum, ensuring logical flow and ease of indexing for the **RAG Chatbot**.
3.  **Tool-Specific Format:** The book's final output format and style must comply with the specifications and conventions enforced by the generative tool used (**Claude Code/Spec-Kit Plus**) to ensure compatibility and consistency.

---

## Success Criteria (The 'What to Deliver')
1.  **Functional RAG Chatbot:** A fully operational RAG Chatbot (**FastAPI, Agents/ChatKit**) that can accurately query and respond based on the content of the AI-native textbook.
2.  **VLA-Integrated Control:** Successful demonstration of an integrated **Vision-Language-Action (VLA)** model performing cognitive planning and high-level control of a simulated humanoid system.
3.  **Complete Textbook:** A comprehensive, four-module AI-native textbook covering: **ROS 2**, **Digital Twin (Simulation)**, **AI-Robot Brain (Isaac)**, and **VLA Integration**.

---

## Constraints (The 'Boundaries')
1.  **Tool Adherence:** Advice is limited to and must utilize the specified tool stack: **ROS 2**, **NVIDIA Isaac Platform**, **Claude Code/Spec-Kit Plus**, and **OpenAI Agents/ChatKit SDKs**.
2.  **Scope Limitation:** Guidance is strictly limited to the technical scope of the four course modules and the resulting humanoid robotics system. Avoid providing generic LLM or non-robotics advice.

## Response snapshot

I have successfully updated the project constitution to version 1.0.0. The new constitution for the "Physical AI & Humanoid Robotics Textbook" is now active.

I also updated the `plan-template.md` to reflect the new constitutional principles. The `spec-template.md` and `tasks-template.md` did not require any changes.

Suggested commit message:
`docs: amend constitution to v1.0.0 (principle additions + governance update)`

## Outcome

- ✅ Impact: Major. The project now has a clear set of principles, standards, success criteria, and constraints.
- 🧪 Tests: N/A
- 📁 Files: 
  - `.specify/memory/constitution.md` (updated)
  - `.specify/templates/plan-template.md` (updated)
- 🔁 Next prompts: The user can now proceed with other commands like `/sp.plan` or `/sp.specify`.
- 🧠 Reflection: The process of updating the constitution and propagating the changes to the templates was successful.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): N/A