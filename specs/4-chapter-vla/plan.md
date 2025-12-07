# Implementation Plan: Chapter 4: Vision-Language-Action (VLA)

**Branch**: `4-chapter-vla` | **Date**: 2025-12-07 | **Spec**: [specs/4-chapter-vla/spec.md](specs/4-chapter-vla/spec.md)
**Input**: Feature specification from `specs/4-chapter-vla/spec.md`

## Summary

This plan outlines the creation of Chapter 4, "Vision-Language-Action (VLA)", a capstone tutorial for expert AI and Robotics Integrators. This chapter will guide readers through building a complete autonomous agent by integrating a voice-command interface (Whisper), a cognitive planning layer (Gemini), and the previously developed robotics systems (ROS 2, Isaac Sim, Nav2).

## Chapter Structure and Content Outline

This chapter will be broken down into the following sections, each corresponding to a markdown file.

### 1. `docs/module4-vla-capstone/index.md`: Introduction to the Complete VLA Architecture
-   **Objective**: Present the final, unified system architecture.
-   **Content**:
    -   Explain the complete autonomous loop: Microphone → Whisper → ROS 2 Text Topic → Gemini Planner Node → ROS 2 Action Sequence → Robot Control.
    -   Emphasize the integration of all previous modules.
    -   Provide a detailed architecture diagram of the final VLA system.

### 2. `docs/module4-vla-capstone/m4-1-voice-to-action.md`: The Speech Processing Front-End
-   **Objective**: Teach the reader how to implement the voice command interface.
-   **Content**:
    -   Step-by-step instructions for creating a "Whisper Listener" ROS 2 node in Python.
    -   Code snippets for using the Whisper API/SDK to transcribe real-time audio.
    -   Instructions on how to publish the transcribed text to a `/command_text` ROS 2 topic.

### 3. `docs/module4-vla-capstone/m4-2-gemini-cognitive-planner.md`: The Cognitive Core with Gemini
-   **Objective**: Guide the reader through building the cognitive planning node.
-   **Content**:
    -   Instructions for creating a ROS 2 node that subscribes to the `/command_text` topic.
    -   Detailed explanation of prompt engineering and function calling with the Gemini API to translate natural language commands into a list of ROS 2 actions.
    -   A Python function example showing the Gemini API call and the parsing of the response into a structured plan (e.g., `["navigate(kitchen)", "find_object(cup)", "grasp_object()"]`).
    -   Publishing the generated plan as a queue of ROS 2 Action Goals.

### 4. `docs/module4-vla-capstone/m4-3-capstone-integration.md`: Capstone Project Integration
-   **Objective**: Guide the reader through the final system integration.
-   **Content**:
    -   Instructions for creating a master coordination node or a ROS 2 Behavior Tree.
    -   Code snippets for an action client that executes the plan sequence from the Gemini planner.
    -   Interfacing with the Nav2 stack from Module 3 for navigation.
    -   Calling computer vision services from Module 3 for object identification.
    -   Invoking manipulation actions from Module 1 for grasping.
    -   A complete walkthrough of the capstone project: "Clean the room".

## Technical Specifications and Assumptions

-   **Primary Tools**: OpenAI Whisper API, Gemini API/SDK.
-   **Integration**: The chapter will heavily rely on the successful completion of Modules 1, 2, and 3.
-   **Security**: API keys will be managed via environment variables as per the spec clarification.
-   **Error Handling**: The system will provide audible feedback for transcription and planning failures.
-   **Latency**: The target end-to-end latency from voice command to robot action is 1-3 seconds.

## Constitution Check

-   **AI-Native Documentation**: **[PASS]**
-   **Actionable Knowledge Base**: **[PASS]**
-   **Technical Accuracy Standard**: **[PASS]**
-   **Modular Structure Standard**: **[PASS]**
-   **Documentation Platform Standard**: **[PASS]**
