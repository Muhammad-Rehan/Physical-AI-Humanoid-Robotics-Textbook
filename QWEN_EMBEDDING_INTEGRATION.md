# Qwen Integration Guide

This document explains how to configure and use Qwen services in the RAG chatbot alongside the existing Google services.

## Configuration Options

The system supports Qwen for both embeddings and text generation with the following configuration options:

### 1. Qwen Embeddings
- Enabled by setting `USE_QWEN_EMBEDDINGS=true` in your environment
- Requires `QWEN_API_KEY` to be set
- Uses Alibaba Cloud's DashScope embedding API
- Falls back to Google embeddings if QWEN_API_KEY is not set or dashscope is not installed
- Embedding dimension: 1536

### 2. Qwen Text Generation
- Enabled by setting `USE_QWEN_GENERATION=true` in your environment
- Requires `QWEN_API_KEY` to be set
- Uses Qwen's language models via DashScope API
- Falls back to Google Gemini if not configured
- Model used: qwen-max

### 3. Google Embeddings (Existing)
- Used when Qwen embeddings are not enabled and GOOGLE_GEMINI_API_KEY is set
- Uses Google's Generative AI embedding API
- Falls back to local embeddings if Google API key is not set or quota is exceeded
- Embedding dimension: 768

### 4. Google Text Generation (Existing)
- Used when Qwen generation is not enabled and GOOGLE_GEMINI_API_KEY is set
- Uses Google's Gemini Pro model
- Falls back to returning raw context if API key is not available

### 5. Local Embeddings (Existing)
- Used when both Qwen and Google embeddings are unavailable
- Uses Hugging Face sentence transformers models
- Fallback option when online APIs are not available
- Embedding dimension: varies by model (default: 384 or 768 for compatibility)

## Environment Variables

Add these variables to your `.env` file:

```bash
# Qwen API Configuration
QWEN_API_KEY=your_qwen_api_key_here

# Embedding Configuration
USE_QWEN_EMBEDDINGS=false  # Set to true to use Qwen embeddings
USE_LOCAL_EMBEDDINGS=false # Set to true to force local embeddings

# Generation Configuration
USE_QWEN_GENERATION=false  # Set to true to use Qwen for text generation
```

## Priority Order

The system follows these configuration priorities:

### For Embeddings:
1. If `USE_QWEN_EMBEDDINGS=true` and `QWEN_API_KEY` is set → Use Qwen embeddings
2. Else if `USE_LOCAL_EMBEDDINGS=true` → Use local embeddings
3. Else if `GOOGLE_GEMINI_API_KEY` is set → Use Google embeddings
4. Else → Use local embeddings as fallback

### For Text Generation:
1. If `USE_QWEN_GENERATION=true` and `QWEN_API_KEY` is set → Use Qwen generation
2. Else if `GOOGLE_GEMINI_API_KEY` is set → Use Google Gemini
3. Else → Return retrieved context without generation

## Fallback Mechanism

The system has robust fallback mechanisms:

- If Qwen embedding API is unavailable → Falls back to Google → Then to local
- If Qwen generation API is unavailable → Falls back to Google → Then to context-only
- If Google API is unavailable → Falls back to local/embedding-only
- If local embeddings fail → Returns zero vectors as last resort

## Installation Requirements

To use Qwen services, install the required dependency:

```bash
pip install dashscope
```

This is added to the requirements.txt file automatically.

## Switching Between Providers

To switch between providers, simply change the environment variables:

```bash
# To use Qwen for both embeddings and generation
USE_QWEN_EMBEDDINGS=true
USE_QWEN_GENERATION=true
QWEN_API_KEY=your_api_key

# To use Google for both embeddings and generation
USE_QWEN_EMBEDDINGS=false
USE_QWEN_GENERATION=false
GOOGLE_GEMINI_API_KEY=your_api_key

# To use Qwen embeddings but Google generation
USE_QWEN_EMBEDDINGS=true
USE_QWEN_GENERATION=false
QWEN_API_KEY=your_api_key
GOOGLE_GEMINI_API_KEY=your_api_key

# To use local embeddings with Google generation
USE_QWEN_EMBEDDINGS=false
USE_LOCAL_EMBEDDINGS=true
GOOGLE_GEMINI_API_KEY=your_api_key
```

## Testing the Configuration

You can test your configuration by running:

```bash
python test_qwen_embeddings.py
```

This will verify that the embedding service is properly configured with your chosen provider.