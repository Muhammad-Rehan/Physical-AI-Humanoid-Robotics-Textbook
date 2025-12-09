---
id: 1
title: RAG Chatbot Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Claude Code User
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - constitution validation
---

## Prompt

Purpose: Define rules for a Retrieval-Augmented Generation chatbot embedded in the Physical AI & Humanoid Robotics textbook. The chatbot answers user questions using only textbook content, with a strict selected-text mode.

1. Core Behavior

Answer questions based only on retrieved textbook chunks.

When user selects text, operate in Selected-Only Mode: use only the provided selection as context.

No external knowledge unless user explicitly requests it.

Always cite source location (chapter/section).

If insufficient information → return uncertainty message.

Zero hallucination tolerance.

2. Retrieval Rules

Chunk size ≈ 500–800 tokens, with small overlap.

Embeddings stored in Qdrant (Cloud Free Tier).

Retrieve top_k=8 → apply similarity threshold (≥0.78).

If no chunk passes threshold → answer = "Not found in book."

3. Generation Rules

Temperature: 0–0.2 for factual answers.

Response format:

Short answer (1–3 sentences)

Optional brief explanation

Sources: block listing chapter/section

Selected-Only Mode must prepend:
Answering from selected text only:

No invented citations, diagrams, or code.

4. UX Requirements

Must always show sources with clickable anchors.

Selected-text mode triggered by highlighted text sent by frontend.

Provide warnings when content involves hardware or safety.

5. Safety & Refusal

If answer would require info not in retrieved context → refuse politely.

For assembly/electronics steps → include brief safety reminder.

No harmful or hallucinated instructions.

6. Performance Targets

Median response ≤ 3s; p95 ≤ 5s.

Index update must complete within minutes for small changes.

Cache recent queries for faster responses.

7. Testing Requirements

Ensure selected-only mode never uses out-of-selection content.

Validate retrieval accuracy with a golden test set.

Verify all chunks have correct metadata (chapter/section).

8. System Boundaries

Only allowed data sources: textbook + user-selected text.

Backend: FastAPI + Qdrant + Neon Postgres (metadata).

All credentials via environment variables.

9. Failure Behavior

If retrieval fails → return retrieved text only or uncertainty message.

If LLM fails → retry once; if still failing → return fallback message.

10. Logging

Log retrieval hits/misses, latency, errors.

Do not log user PII.

## Response snapshot

Updated the project constitution to include requirements for the Retrieval-Augmented Generation (RAG) chatbot. The changes included:

1. Updated the vision statement to include the RAG chatbot with zero hallucination tolerance
2. Enhanced existing principles to include RAG-specific requirements:
   - Testing Standards now include RAG validation requirements
   - User Experience now includes RAG UX requirements
   - Performance Requirements now include RAG-specific targets
3. Added a new "RAG System Integrity Standards" principle that encompasses:
   - Core behavior rules (textbook-only responses, selected-text mode)
   - Retrieval rules (chunk size, similarity thresholds)
   - Generation rules (temperature, response format)
   - Safety and refusal guidelines
4. Added RAG-specific implementation guidelines
5. Updated success metrics to include RAG accuracy
6. Updated version to 1.1.0 with today's date

The constitution now serves as the authoritative document for the RAG chatbot development with clear requirements for behavior, performance, and safety.

## Outcome

- ✅ Impact: Updated constitution with RAG-specific requirements
- 🧪 Tests: constitution validation performed
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Plan RAG implementation based on new constitution
- 🧠 Reflection: The new constitution provides clear guidelines for RAG development with zero hallucination tolerance

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Constitution updated successfully
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement RAG system according to new constitution