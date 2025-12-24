# GLM Integration Guide for RAG Chatbot

This document explains how to configure and use GLM (Zhipu AI) services in the RAG chatbot alongside the existing Cohere, Google, and Qwen services.

## Configuration Options

The system supports GLM for text generation with the following configuration options:

### 1. GLM Text Generation
- Enabled by setting `USE_GLM_GENERATION=true` in your environment
- Requires `GLM_API_KEY` to be set
- Uses Zhipu AI's GLM-4 model via their API
- Falls back to Qwen generation if GLM_API_KEY is not set or zhipuai is not installed
- Priority: Highest among generation models when enabled

### 2. Cohere Embeddings (Updated)
- Enabled by setting `USE_COHERE_EMBEDDINGS=true` in your environment
- Requires `COHERE_API_KEY` to be set
- Uses Cohere's embedding API (v3 models)
- Falls back to Qwen embeddings if COHERE_API_KEY is not set or cohere is not installed
- Embedding dimension: 1024 for multilingual-v3.0, 384 for embed-multilingual-light-v3.0, 1024 for embed-english-v3.0, 1024 for embed-english-light-v3.0

### 3. Qwen Text Generation (Existing)
- Enabled by setting `USE_QWEN_GENERATION=true` and GLM disabled
- Requires `QWEN_API_KEY` to be set
- Uses Qwen's language models via DashScope API
- Falls back to Google Gemini if not configured

### 4. Google Text Generation (Existing)
- Uses Google's Gemini Pro model
- Falls back to returning raw context if API key is not available

### 5. Cohere Embeddings (Existing)
- Enabled by setting `USE_COHERE_EMBEDDINGS=true` in your environment
- Requires `COHERE_API_KEY` to be set
- Uses Cohere's embedding API (v3 models)
- Falls back to Qwen embeddings if COHERE_API_KEY is not set or cohere is not installed
- Embedding dimension: 1024 for multilingual-v3.0, 384 for embed-multilingual-light-v3.0

## Environment Variables

Add these variables to your `.env` file:

```bash
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
USE_GLM_GENERATION=false     # Set to true to use GLM for text generation (highest priority)
USE_QWEN_GENERATION=false    # Set to true to use Qwen for text generation (second priority)
```

## Priority Order

The system follows these configuration priorities for text generation:

1. If `USE_GLM_GENERATION=true` and `GLM_API_KEY` is set → Use GLM generation
2. If `USE_QWEN_GENERATION=true` and `QWEN_API_KEY` is set → Use Qwen generation
3. Else if `GOOGLE_GEMINI_API_KEY` is set → Use Google Gemini
4. Else → Return retrieved context without generation

For embeddings:
1. If `USE_COHERE_EMBEDDINGS=true` and `COHERE_API_KEY` is set → Use Cohere embeddings
2. Else if `USE_QWEN_EMBEDDINGS=true` and `QWEN_API_KEY` is set → Use Qwen embeddings
3. Else if `USE_LOCAL_EMBEDDINGS=true` → Use local embeddings
4. Else if `GOOGLE_GEMINI_API_KEY` is set → Use Google embeddings
5. Else → Use local embeddings as fallback

## Fallback Mechanism

The system has robust fallback mechanisms:

- If GLM generation API is unavailable → Falls back to Qwen → Google → Then to context-only
- If Qwen generation API is unavailable → Falls back to Google → Then to context-only
- If Qwen embedding API is unavailable → Falls back to Google → Then to local
- If Google API is unavailable → Falls back to local/embedding-only
- If local embeddings fail → Returns zero vectors as last resort

## Installation Requirements

To use GLM services, install the required dependency:

```bash
pip install zhipuai
```

This is already included in the requirements.txt file.

## Available GLM Models

- `glm-4`: The main model for text generation

## Switching Between Providers

To switch between providers, simply change the environment variables:

```bash
# To use GLM for generation with Cohere embeddings
USE_GLM_GENERATION=true
USE_QWEN_GENERATION=false
USE_COHERE_EMBEDDINGS=true
GLM_API_KEY=your_api_key
COHERE_API_KEY=your_cohere_api_key

# To use Qwen for both generation and embeddings
USE_GLM_GENERATION=false
USE_QWEN_GENERATION=true
QWEN_API_KEY=your_api_key

# To use Google for generation with Cohere embeddings
USE_GLM_GENERATION=false
USE_QWEN_GENERATION=false
GOOGLE_GEMINI_API_KEY=your_api_key
USE_COHERE_EMBEDDINGS=true
```

## Testing the Configuration

You can test your configuration by running:

```bash
python test_glm_integration.py
```

This will verify that the GLM service is properly configured with your chosen provider.