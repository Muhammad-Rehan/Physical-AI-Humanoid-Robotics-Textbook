# Local Embeddings Implementation

## Overview
This implementation provides a flexible embedding service that can use either Google's embedding API or local embeddings. When Google API quota is exceeded, the system automatically falls back to local embeddings.

## Files Created/Modified

### 1. backend/src/services/local_embedding_service.py
- Implements local embedding generation using Sentence Transformers
- Uses the `all-MiniLM-L6-v2` model by default (384-dimensional vectors)
- Includes fallback mechanisms when model initialization fails
- Supports configurable models via environment variables

### 2. backend/src/services/embedding_service.py
- Updated to support both local and Google embeddings
- Automatically falls back to local embeddings when Google API fails
- Environment variable controlled via `USE_LOCAL_EMBEDDINGS=true`

### 3. backend/requirements.txt
- Added dependencies: `transformers`, `torch`, `sentence-transformers`, `numpy`

### 4. backend/.env.example
- Example configuration file showing how to enable local embeddings

## Configuration

### To use local embeddings:
Set the environment variable in your `.env` file:
```
USE_LOCAL_EMBEDDINGS=true
```

### To specify a different local model:
```
USE_LOCAL_EMBEDDINGS=true
LOCAL_EMBEDDING_MODEL=all-mpnet-base-v2  # Higher quality but slower
```

### To continue using Google embeddings (default):
```
USE_LOCAL_EMBEDDINGS=false
GOOGLE_GEMINI_API_KEY=your_api_key_here
```

## Fallback Behavior
The system automatically falls back to local embeddings when:
- `GOOGLE_GEMINI_API_KEY` is not set
- Google API quota is exceeded (HTTP 429)
- Any other Google API error occurs

## Model Options
- `all-MiniLM-L6-v2` (default): Fast, 384 dimensions, good for most use cases
- `all-mpnet-base-v2`: Higher quality, 768 dimensions, slower
- `all-distilroberta-v1`: Good balance, 768 dimensions

## Testing
Run the test script after installing dependencies:
```bash
pip install -r backend/requirements.txt
python backend/test_local_embeddings.py
```

## Benefits
- No API quota limitations when using local embeddings
- Works offline after initial model download
- Maintains compatibility with existing Qdrant vector database
- Automatic fallback when Google API is unavailable