# Cohere Integration Guide

This document explains how to configure and use Cohere services in the RAG chatbot alongside the existing Google and Qwen services.

## Configuration Options

The system supports Cohere for both embeddings and text generation with the following configuration options:

### 1. Cohere Embeddings
- Enabled by setting `USE_COHERE_EMBEDDINGS=true` in your environment
- Requires `COHERE_API_KEY` to be set
- Uses Cohere's embedding API (v3 models)
- Falls back to Google embeddings if COHERE_API_KEY is not set or cohere is not installed
- Embedding dimension: 1024 for multilingual-v3.0, 384 for embed-multilingual-light-v3.0, 1024 for embed-english-v3.0, 1024 for embed-english-light-v3.0

### 2. Cohere Text Generation
- Enabled by setting `USE_COHERE_GENERATION=true` in your environment
- Requires `COHERE_API_KEY` to be set
- Uses Cohere's language models via API
- Falls back to Google Gemini if not configured
- Default model: command-r-plus

### 3. Google Embeddings (Existing)
- Used when Cohere embeddings are not enabled and GOOGLE_GEMINI_API_KEY is set
- Uses Google's Generative AI embedding API
- Falls back to local embeddings if Google API key is not set or quota is exceeded
- Embedding dimension: 768

### 4. Google Text Generation (Existing)
- Used when Cohere generation is not enabled and GOOGLE_GEMINI_API_KEY is set
- Uses Google's Gemini Pro model
- Falls back to returning raw context if API key is not available

### 5. Qwen Embeddings (Existing)
- Used when Cohere and Google embeddings are not enabled but QWEN_API_KEY is set
- Uses Alibaba Cloud's DashScope embedding API
- Falls back to Google embeddings if QWEN_API_KEY is not set or dashscope is not installed
- Embedding dimension: 1536

### 6. Local Embeddings (Existing)
- Used when Cohere, Qwen and Google embeddings are unavailable
- Uses Hugging Face sentence transformers models
- Fallback option when online APIs are not available
- Embedding dimension: varies by model (default: 384 or 768 for compatibility)

## Environment Variables

Add these variables to your `.env` file:

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Embedding Configuration
USE_COHERE_EMBEDDINGS=false  # Set to true to use Cohere embeddings
USE_QWEN_EMBEDDINGS=false   # Set to true to use Qwen embeddings
USE_LOCAL_EMBEDDINGS=false  # Set to true to force local embeddings

# Generation Configuration
USE_COHERE_GENERATION=false  # Set to true to use Cohere for text generation
USE_QWEN_GENERATION=false    # Set to true to use Qwen for text generation

# Cohere Model Selection (Optional - defaults to multilingual-v3.0)
COHERE_EMBED_MODEL=multilingual-v3.0  # Options: multilingual-v3.0, embed-english-v3.0, embed-english-light-v3.0, embed-multilingual-light-v3.0
COHERE_GENERATION_MODEL=command-r-plus  # Options: command-r-plus, command-nightly, etc.
```

## Priority Order

The system follows these configuration priorities:

### For Embeddings:
1. If `USE_COHERE_EMBEDDINGS=true` and `COHERE_API_KEY` is set → Use Cohere embeddings
2. Else if `USE_QWEN_EMBEDDINGS=true` and `QWEN_API_KEY` is set → Use Qwen embeddings
3. Else if `USE_LOCAL_EMBEDDINGS=true` → Use local embeddings
4. Else if `GOOGLE_GEMINI_API_KEY` is set → Use Google embeddings
5. Else → Use local embeddings as fallback

### For Text Generation:
1. If `USE_COHERE_GENERATION=true` and `COHERE_API_KEY` is set → Use Cohere generation
2. If `USE_QWEN_GENERATION=true` and `QWEN_API_KEY` is set → Use Qwen generation
3. Else if `GOOGLE_GEMINI_API_KEY` is set → Use Google Gemini
4. Else → Return retrieved context without generation

## Fallback Mechanism

The system has robust fallback mechanisms:

- If Cohere embedding API is unavailable → Falls back to Qwen → Google → Then to local
- If Cohere generation API is unavailable → Falls back to Qwen → Google → Then to context-only
- If Qwen API is unavailable → Falls back to Google → Then to local
- If Google API is unavailable → Falls back to local/embedding-only
- If local embeddings fail → Returns zero vectors as last resort

## Installation Requirements

To use Cohere services, install the required dependency:

```bash
pip install cohere
```

Add this to the requirements.txt file.

## Available Embedding Models and Dimensions

- `multilingual-v3.0`: 1024 dimensions
- `embed-english-v3.0`: 1024 dimensions
- `embed-english-light-v3.0`: 384 dimensions
- `embed-multilingual-light-v3.0`: 384 dimensions

## Switching Between Providers

To switch between providers, simply change the environment variables:

```bash
# To use Cohere for both embeddings and generation
USE_COHERE_EMBEDDINGS=true
USE_COHERE_GENERATION=true
COHERE_API_KEY=your_api_key

# To use Qwen for both embeddings and generation
USE_COHERE_EMBEDDINGS=false
USE_COHERE_GENERATION=false
USE_QWEN_EMBEDDINGS=true
USE_QWEN_GENERATION=true
QWEN_API_KEY=your_api_key

# To use Google for both embeddings and generation
USE_COHERE_EMBEDDINGS=false
USE_COHERE_GENERATION=false
USE_QWEN_EMBEDDINGS=false
USE_QWEN_GENERATION=false
GOOGLE_GEMINI_API_KEY=your_api_key

# To use Cohere embeddings but Google generation
USE_COHERE_EMBEDDINGS=true
USE_COHERE_GENERATION=false
COHERE_API_KEY=your_api_key
GOOGLE_GEMINI_API_KEY=your_api_key

# To use Cohere embeddings but Qwen generation
USE_COHERE_EMBEDDINGS=true
USE_COHERE_GENERATION=false
COHERE_API_KEY=your_api_key
USE_QWEN_GENERATION=true
QWEN_API_KEY=your_api_key

# To use Cohere embedding model selection
USE_COHERE_EMBEDDINGS=true
COHERE_API_KEY=your_api_key
COHERE_EMBED_MODEL=multilingual-v3.0  # or embed-english-v3.0, etc.
```

## Testing the Configuration

You can test your configuration by running:

```bash
python test_cohere_embeddings.py
```

This will verify that the embedding service is properly configured with your chosen provider.