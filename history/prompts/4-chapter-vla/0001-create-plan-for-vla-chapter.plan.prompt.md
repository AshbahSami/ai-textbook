---
id: 0001
title: Create Plan for VLA Chapter
stage: plan
date: 2025-12-07
surface: agent
model: gemini-pro
feature: 4-chapter-vla
branch: 4-chapter-vla
user: unspecified
command: /sp.plan
labels: ["plan", "vla", "gemini"]
links:
  spec: specs/4-chapter-vla/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/4-chapter-vla/plan.md
tests:
  - None
---

## Prompt

/sp.plan

**Module Topic:** Module 4: Vision-Language-Action (VLA)

### 1. Architecture Overview (The Complete Autonomous Loop)
The module must start by presenting the final, complete system architecture.
* **Focus:** Define the full loop: **Microphone $\rightarrow$ Whisper $\rightarrow$ ROS 2 Text Topic $\rightarrow$ Gemini Planner Node $\rightarrow$ ROS 2 Action Sequence $\rightarrow$ Robot Control.**
* **Goal:** Emphasize that this module is about tying all previously built systems into a unified autonomous agent. 

---

### 2. Implementation Phases (Sequential Build Steps)
The instructional content will be built logically, integrating components from M1, M2, and M3.

1.  **Phase 1: Speech Processing Front-End:** Instructions for setting up the **Whisper Listener Node** to listen for audio input and publish the resulting text string to a new ROS 2 topic (e.g., `/command_text`). (Corresponds to `m4-1-voice-to-action.md`).
2.  **Phase 2: Gemini Cognitive Planner Node:** Detail the Python code for a ROS 2 node that subscribes to `/command_text`, uses **Gemini's reasoning capabilities** to generate a sequence of high-level actions, and outputs this plan as a queue of ROS 2 Action Goals. (Corresponds to `m4-2-gemini-cognitive-planner.md`).
3.  **Phase 3: Capstone Integration:** Guide the student through creating the final **ROS 2 Behavior Tree** or master coordination node that executes the sequence, interfacing with the Nav2 stack (M3), computer vision services (M3), and manipulation services (M1). (Corresponds to `m4-3-capstone-integration.md`).

---

### 3. Component Breakdown (Required Code & Configuration)
The guide must provide specific, ready-to-use code blocks for:

* **Whisper Listener Script:** Python code utilizing the Whisper API/SDK.
* **Gemini Planner Function:** Python function showing the prompt and function calling structure for translating natural language into a list of ROS 2 actions (e.g., `["navigate(kitchen)", "find_object(cup)", "grasp_object()"]`).
* **ROS 2 Action Client/Server Logic:** Code snippets for the master node that manages the flow of the autonomous task.

---

### 4. File Structure (Output Mapping)
The content generated will be mapped directly to the following Docusaurus file structure:

| Docusaurus Path | Content Focus |
| :--- | :--- |
| **`docs/module4-vla-capstone/index.md`** | **Introduction** and **The Complete VLA Architecture** |
| **`docs/module4-vla-capstone/m4-1-voice-to-action.md`** | Whisper ASR and Text Command Parsing |
| **`docs/module4-vla-capstone/m4-2-gemini-cognitive-planner.md`** | LLM Planning and ROS 2 Sequence Generation (using Gemini) |
| **`docs/module4-vla-capstone/m4-3-capstone-integration.md`** | Capstone Project Walkthrough and Final System Integration |

## Response snapshot

I have created a comprehensive `plan.md` file that outlines the structure and content for Chapter 4: "Vision-Language-Action (VLA)". The plan is tailored for a book chapter and includes a detailed breakdown of the sections, technical specifications, and a validation against the project's constitution.