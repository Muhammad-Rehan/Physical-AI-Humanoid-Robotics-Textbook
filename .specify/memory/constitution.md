<!-- Sync Impact Report:
Version change: 1.0.0 -> 1.1.0
List of modified principles:
  - Code Quality Standards -> Code Quality Standards
  - Testing Standards -> Testing Standards with RAG Focus
  - User Experience Consistency -> User Experience with RAG Requirements
  - Performance Requirements -> Performance Requirements with RAG Targets
  - New -> RAG System Integrity Standards
Added sections: Vision Statement, Implementation Guidelines, Success Metrics
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .specify/templates/commands/sp.constitution.md: ⚠ pending
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Vision Statement
To create an AI-native educational resource that empowers learners to master Physical AI and Humanoid Robotics through accessible, interactive, and technically accurate content that bridges theory with practical application, enhanced by a Retrieval-Augmented Generation chatbot that provides precise, textbook-grounded answers with zero hallucination tolerance.

## Core Principles

### 1. Code Quality Standards
All code examples shall follow clean code principles with clear naming conventions
Implement modular architecture with logical separation of concerns
Use consistent indentation, formatting, and documentation style throughout
Provide both simplified educational examples and production-ready implementations

Every code snippet must include comprehensive comments explaining the "why" not just the "what"
Provide inline documentation for complex algorithms and implementations
Include expected outputs and common pitfalls for each code example
Maintain a consistent API documentation format for all custom components

Implement semantic versioning for all code examples and libraries
Maintain clear commit messages following conventional commit standards
Document breaking changes with migration guides
Tag all example code with compatible ROS 2, Gazebo, and Isaac Sim versions

### 2. Testing Standards with RAG Focus
All code examples must be tested in the specified environments (Ubuntu 22.04, ROS 2 Humble/Iron)
Provide automated tests for critical components and algorithms
Include hardware compatibility matrices for different robot platforms
Document performance benchmarks for resource-intensive operations

Content must be reviewed by subject matter experts in robotics and AI
Implement a feedback mechanism for continuous improvement based on learner experiences
Validate learning outcomes through practical assessments and projects
Ensure progressive complexity that builds on previous concepts

Test all RAG chatbot responses against textbook content for accuracy
Verify that code examples work across different hardware specifications
Ensure seamless integration between Docusaurus frontend and backend components
Perform cross-browser and device compatibility testing

Validate retrieval accuracy with a golden test set
Ensure selected-only mode never uses out-of-selection content
Verify all chunks have correct metadata (chapter/section)
Test RAG response format consistency and source citation accuracy

### 3. User Experience with RAG Requirements
Implement a consistent information architecture throughout the textbook
Maintain uniform navigation patterns and content structure
Provide multiple learning pathways for different backgrounds (software vs. hardware)
Ensure all content follows the established pedagogical framework

Comply with WCAG 2.1 AA accessibility guidelines
Provide alternative text descriptions for all visual content
Ensure keyboard navigation support for all interactive elements
Implement responsive design for various screen sizes and devices

Combine textual explanations with visual diagrams, code examples, and videos
Provide interactive simulations where appropriate to demonstrate concepts
Include practical exercises that reinforce theoretical knowledge
Design the RAG chatbot to provide contextual assistance tailored to the current topic

Must always show sources with clickable anchors
Selected-text mode triggered by highlighted text sent by frontend
Provide warnings when content involves hardware or safety
Design intuitive interfaces for RAG interactions with clear feedback

### 4. Performance Requirements with RAG Targets
Optimize all assets for fast loading without compromising quality
Implement lazy loading for resource-intensive content
Ensure the RAG chatbot provides responses within 3 seconds for 95% of queries
Minimize dependencies to reduce installation complexity for learners

Provide clear hardware requirements for all examples and projects
Optimize code examples for performance on specified hardware (RTX GPUs, Jetson platforms)
Include performance profiling for resource-intensive operations
Document expected resource utilization for all major components

Design the RAG system to handle concurrent users efficiently
Implement caching strategies for frequently accessed content
Ensure the architecture supports future content expansion
Design for both local and cloud deployment scenarios

Median response time ≤ 3s; p95 ≤ 5s for RAG queries
Index update must complete within minutes for small changes
Cache recent queries for faster responses
Optimize retrieval performance for Qdrant vector database

### 5. RAG System Integrity Standards
Answer questions based only on retrieved textbook chunks
When user selects text, operate in Selected-Only Mode: use only the provided selection as context
No external knowledge unless user explicitly requests it
Always cite source location (chapter/section)

If insufficient information → return uncertainty message
Zero hallucination tolerance
Chunk size ≈ 500–800 tokens, with small overlap
Embeddings stored in Qdrant (Cloud Free Tier)

Retrieve top_k=8 → apply similarity threshold (≥0.78)
If no chunk passes threshold → answer = "Not found in book"
Temperature: 0–0.2 for factual answers
Response format: Short answer (1–3 sentences) + optional brief explanation + Sources block

Selected-Only Mode must prepend: "Answering from selected text only:"
No invented citations, diagrams, or code
If answer would require info not in retrieved context → refuse politely
For assembly/electronics steps → include brief safety reminder

## Implementation Guidelines
Development Workflow
All content must be created using Claude Code and structured with Spec-Kit Plus
Implement continuous integration for automated testing and validation
Conduct regular reviews to ensure compliance with this constitution
Maintain a change log documenting all significant modifications
Quality Assurance
Establish a review process for all content before publication
Implement automated checks for code quality and performance
Conduct user testing with representative learners
Monitor and address issues reported through the feedback system

RAG-Specific Implementation
Backend: FastAPI + Qdrant + Neon Postgres (metadata)
All credentials via environment variables
Only allowed data sources: textbook + user-selected text
Implement proper error handling for retrieval and LLM failures

## Success Metrics
Content Quality: Measured through expert reviews and technical accuracy
User Engagement: Assessed through interaction patterns with the RAG chatbot
Learning Outcomes: Evaluated through project completion rates and assessment scores
System Performance: Monitored through response times and resource utilization metrics
RAG Accuracy: Evaluated through retrieval precision and response correctness

## Governance
This constitution serves as the foundational document for the "Physical AI & Humanoid Robotics Textbook" project. All development, content creation, and system operations must strictly adhere to the principles and guidelines outlined herein. Amendments to this constitution require careful consideration, documentation, and approval from relevant stakeholders. Compliance will be regularly reviewed to ensure the highest standards of technical excellence and user experience.

**Version**: 1.1.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-09
