# OpenRouter Integration Guide for RAG Chatbot

This document explains how to configure and use OpenRouter services in the RAG chatbot alongside the existing Cohere, GLM, Google, and Qwen services.

## Configuration Options

The system supports OpenRouter for text generation with the following configuration options:

### 1. OpenRouter Text Generation
- Enabled by setting `USE_OPENROUTER_GENERATION=true` in your environment
- Requires `OPENROUTER_API_KEY` to be set
- Uses OpenRouter's API via their OpenAI-compatible interface
- Falls back to GLM generation if OPENROUTER_API_KEY is not set or openrouter is not installed
- Priority: Highest among generation models when enabled

### 2. GLM Text Generation (Existing)
- Enabled by setting `USE_GLM_GENERATION=true` and OpenRouter disabled
- Requires `GLM_API_KEY` to be set
- Uses Zhipu AI's GLM models via their API
- Falls back to Qwen generation if GLM_API_KEY is not set or zhipuai is not installed

### 3. Cohere Embeddings (Existing)
- Enabled by setting `USE_COHERE_EMBEDDINGS=true` in your environment
- Requires `COHERE_API_KEY` to be set
- Uses Cohere's embedding API (v3 models)
- Falls back to Qwen embeddings if COHERE_API_KEY is not set or cohere is not installed
- Embedding dimension: 1024 for multilingual-v3.0, 384 for embed-multilingual-light-v3.0

## Environment Variables

Add these variables to your `.env` file:

```bash
# OpenRouter API Configuration
OPENROUTER_API_KEY=your_openrouter_api_key_here

# GLM API Configuration
GLM_API_KEY=your_glm_api_key_here

# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qwen API Configuration
QWEN_API_KEY=your_qwen_api_key_here

# Google API Configuration
GOOGLE_GEMINI_API_KEY=your_google_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_content

# Embedding Configuration
USE_COHERE_EMBEDDINGS=false  # Set to true to use Cohere embeddings
USE_QWEN_EMBEDDINGS=false   # Set to true to use Qwen embeddings
USE_LOCAL_EMBEDDINGS=false  # Set to true to force local embeddings

# Generation Configuration
USE_OPENROUTER_GENERATION=false  # Set to true to use OpenRouter for text generation (highest priority)
USE_GLM_GENERATION=false     # Set to true to use GLM for text generation (second priority)
USE_QWEN_GENERATION=false    # Set to true to use Qwen for text generation (third priority)
```

### OpenRouter Model Configuration
You can specify which model to use with the `OPENROUTER_MODEL` environment variable:

```bash
# Default (free) model for OpenRouter
OPENROUTER_MODEL=microsoft/wizardlm-2-8x22b

# Alternative free model
OPENROUTER_MODEL=openchat/openchat-7b

# Or any other OpenRouter supported model
```

## Priority Order

The system follows these configuration priorities for text generation:

1. If `USE_OPENROUTER_GENERATION=true` and `OPENROUTER_API_KEY` is set → Use OpenRouter generation
2. If `USE_GLM_GENERATION=true` and `GLM_API_KEY` is set → Use GLM generation
3. If `USE_QWEN_GENERATION=true` and `QWEN_API_KEY` is set → Use Qwen generation
4. Else if `GOOGLE_GEMINI_API_KEY` is set → Use Google Gemini
5. Else → Return retrieved context without generation

## Available OpenRouter Models

Some popular models available through OpenRouter:
- `microsoft/wizardlm-2-8x22b` - WizardLM-2 model (good free option)
- `openchat/openchat-7b` - OpenChat model (free option)
- `nousresearch/nous-hermes-2-mistral-7b-dpo` - Nous-Hermes model
- `microsoft/wizardlm-2-7b` - WizardLM-2 7B model (faster, free option)

## Fallback Mechanism

The system has robust fallback mechanisms:

- If OpenRouter generation API is unavailable → Falls back to GLM → Qwen → Google → Then to context-only
- If GLM generation API is unavailable → Falls back to Qwen → Google → Then to context-only
- If Qwen generation API is unavailable → Falls back to Google → Then to context-only
- If Qwen embedding API is unavailable → Falls back to Google → Then to local
- If Google API is unavailable → Falls back to local/embedding-only
- If local embeddings fail → Returns zero vectors as last resort

## Installation Requirements

To use OpenRouter services, install the required dependency (already included):

```bash
pip install openrouter
```

This is already included in the requirements.txt file.

## Getting Started with OpenRouter

1. Sign up at [OpenRouter](https://openrouter.ai/)
2. Get your API key from your dashboard
3. Add it to your `.env` file
4. Set `USE_OPENROUTER_GENERATION=true`
5. Optionally set `OPENROUTER_MODEL` to specify which model to use

## Testing the Configuration

You can test your configuration by running:

```bash
python test_openrouter_integration.py
```

This will verify that the OpenRouter service is properly configured with your chosen provider.