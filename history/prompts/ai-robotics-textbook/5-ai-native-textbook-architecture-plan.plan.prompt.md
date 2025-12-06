---
id: 5
title: AI-Native Textbook Architecture Plan
stage: plan
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: ai-robotics-textbook
branch: 001-ai-robotics-textbook
user: user
command: /sp.plan Project Plan: Tech Stack & Architecture for Physical AI Textbook\nTechnology Stack\nCore Framework\nDocusaurus: The primary framework for building and publishing our textbook as a modern static website\nProvides excellent documentation features out of the box\nSupports Markdown content with React components\nBuilt-in search functionality\nResponsive design for mobile and desktop\nEasy deployment to GitHub Pages\nContent Creation & Management\nClaude Code: AI-powered assistant for generating initial content drafts\nWill help create technical explanations, code examples, and diagrams\nWill assist in maintaining consistency across chapters\nWill help translate complex technical concepts into accessible language\nSpec-Kit Plus: For structuring and organizing the book content\nWill define the hierarchy and relationships between sections\nWill maintain consistency in formatting and structure\nWill help manage the content creation workflow\nVersion Control & Collaboration\nGitHub: For version control and collaboration\nWill store all book content in Markdown files\nWill track changes and contributions\nWill enable deployment through GitHub Actions\nContent Enhancement\nMermaid: For creating technical diagrams and flowcharts\nIntegrated with Docusaurus\nWill visualize ROS 2 architectures, data flows, and system designs\nPrismJS: For syntax highlighting in code examples\nIntegrated with Docusaurus\nWill provide clear, readable code snippets in Python and other languages\nArchitecture Overview\nContent Structure\n/physical-ai-textbook├── docs/                 # Main content directory│   ├── intro/           # Introduction and overview│   ├── module1-ros2/    # Module 1: The Robotic Nervous System│   ├── module2-gazebo/  # Module 2: The Digital Twin│   ├── module3-isaac/   # Module 3: The AI-Robot Brain│   ├── module4-vla/     # Module 4: Vision-Language-Action│   └── resources/       # Additional resources, references├── static/              # Static assets (images, videos)├── src/                 # Custom React components├── docusaurus.config.js # Docusaurus configuration└── sidebars.js          # Navigation structure\nContent Creation Workflow\nOutline Creation: Use Spec-Kit Plus to create a detailed outline of each module and chapter\nContent Generation: Use Claude Code to generate initial drafts based on the outline\nReview & Refinement: Review and refine the content for technical accuracy and clarity\nIntegration: Add the content to the appropriate Markdown files in the docs directory\nEnhancement: Add diagrams, code examples, and other visual elements\nTesting: Review the content in the Docusaurus site to ensure proper formatting and navigation\nCustom Components\nWe'll create custom React components for:\n\nInteractive Code Examples: Code snippets with live execution (where appropriate)\nTechnical Diagrams: Custom components for displaying robot architectures and systems\nChapter Navigation: Enhanced navigation within and between chapters\nLearning Objectives: Structured display of learning goals for each chapter\nDeployment Strategy\nGitHub Actions: Automated build and deployment pipeline\nTrigger on push to main branch\nBuild Docusaurus site\nDeploy to GitHub Pages\nVersion Management: Semantic versioning for book releases\nMajor versions for significant content updates\nMinor versions for new chapters or sections\nPatch versions for corrections and minor improvements\nContent Quality Assurance\nTechnical Accuracy\nCode examples will be tested in specified environments (Ubuntu 22.04, ROS 2 Humble)\nTechnical specifications will be cross-referenced with official documentation\nSubject matter experts will review specialized content\nConsistency\nStyle guide for writing, formatting, and terminology\nTemplate for chapter structure and elements\nAutomated checks for consistent formatting\nAccessibility\nAlt text for all images and diagrams\nSemantic HTML structure\nKeyboard navigation support\nHigh contrast color scheme\nPerformance Optimization\nLazy loading for images and heavy content\nOptimized assets for fast loading\nMinimal JavaScript for improved performance\nSEO optimization for better discoverability
labels: ["plan", "architecture", "tech-stack"]
links:
  spec: E:/Hackathon/specs/001-ai-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - E:/Hackathon/specs/001-ai-robotics-textbook/plan.md
tests:
---

## Prompt

```text
Project Plan: Tech Stack & Architecture for Physical AI Textbook
Technology Stack
Core Framework
Docusaurus: The primary framework for building and publishing our textbook as a modern static website
Provides excellent documentation features out of the box
Supports Markdown content with React components
Built-in search functionality
Responsive design for mobile and desktop
Easy deployment to GitHub Pages
Content Creation & Management
Claude Code: AI-powered assistant for generating initial content drafts
Will help create technical explanations, code examples, and diagrams
Will assist in maintaining consistency across chapters
Will help translate complex technical concepts into accessible language
Spec-Kit Plus: For structuring and organizing the book content
Will define the hierarchy and relationships between sections
Will maintain consistency in formatting and structure
Will help manage the content creation workflow
Version Control & Collaboration
GitHub: For version control and collaboration
Will store all book content in Markdown files
Will track changes and contributions
Will enable deployment through GitHub Actions
Content Enhancement
Mermaid: For creating technical diagrams and flowcharts
Integrated with Docusaurus
Will visualize ROS 2 architectures, data flows, and system designs
PrismJS: For syntax highlighting in code examples
Integrated with Docusaurus
Will provide clear, readable code snippets in Python and other languages
Architecture Overview
Content Structure
/physical-ai-textbook├── docs/                 # Main content directory│   ├── intro/           # Introduction and overview│   ├── module1-ros2/    # Module 1: The Robotic Nervous System│   ├── module2-gazebo/  # Module 2: The Digital Twin│   ├── module3-isaac/   # Module 3: The AI-Robot Brain│   ├── module4-vla/     # Module 4: Vision-Language-Action│   └── resources/       # Additional resources, references├── static/              # Static assets (images, videos)├── src/                 # Custom React components├── docusaurus.config.js # Docusaurus configuration└── sidebars.js          # Navigation structure
Content Creation Workflow
Outline Creation: Use Spec-Kit Plus to create a detailed outline of each module and chapter
Content Generation: Use Claude Code to generate initial drafts based on the outline
Review & Refinement: Review and refine the content for technical accuracy and clarity
Integration: Add the content to the appropriate Markdown files in the docs directory
Enhancement: Add diagrams, code examples, and other visual elements
Testing: Review the content in the Docusaurus site to ensure proper formatting and navigation
Custom Components
We'll create custom React components for:

Interactive Code Examples: Code snippets with live execution (where appropriate)
Technical Diagrams: Custom components for displaying robot architectures and systems
Chapter Navigation: Enhanced navigation within and between chapters
Learning Objectives: Structured display of learning goals for each chapter
Deployment Strategy
GitHub Actions: Automated build and deployment pipeline
Trigger on push to main branch
Build Docusaurus site
Deploy to GitHub Pages
Version Management: Semantic versioning for book releases
Major versions for significant content updates
Minor versions for new chapters or sections
Patch versions for corrections and minor improvements
Content Quality Assurance
Technical Accuracy
Code examples will be tested in specified environments (Ubuntu 22.04, ROS 2 Humble)
Technical specifications will be cross-referenced with official documentation
Subject matter experts will review specialized content
Consistency
Style guide for writing, formatting, and terminology
Template for chapter structure and elements
Automated checks for consistent formatting
Accessibility
Alt text for all images and diagrams
Semantic HTML structure
Keyboard navigation support
High contrast color scheme
Performance Optimization
Lazy loading for images and heavy content
Optimized assets for fast loading\nMinimal JavaScript for improved performance\nSEO optimization for better discoverability
```

## Response snapshot

Branch: 001-ai-robotics-textbook
Plan File: E:/Hackathon/specs/001-ai-robotics-textbook/plan.md

## Outcome

- ✅ Impact: The implementation plan for the AI-Native Textbook for Physical AI & Humanoid Robotics has been successfully created and saved.
- 🧪 Tests: N/A
- 📁 Files: Updated E:/Hackathon/specs/001-ai-robotics-textbook/plan.md.
- 🔁 Next prompts: The next step is to create an Architectural Decision Record (ADR) if architecturally significant decisions were made, or proceed to `/sp.tasks`.
- 🧠 Reflection: Successfully created the plan, bypassing earlier file write issues using a direct bash command. PHR created.

## Evaluation notes (flywheel)

- Failure modes observed: Repeated `Edit` and `Write` tool failures due to `old_string` mismatch or file modification issues. Resolved by using a direct `Bash` overwrite with heredoc.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
