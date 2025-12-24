# Cohere Embedding Integration for Book Data

## Overview
This document explains how to use Cohere embeddings to process and store your Physical AI & Humanoid Robotics Textbook content in Qdrant for semantic search and retrieval.

## Processed Data
Your book content has been successfully processed and embeddings have been generated. The following files have been created:

- `processed_book_data.json` - Contains 183 content chunks with Cohere embeddings from 36 book pages

## Setup Instructions

### Step 1: Run Qdrant
Before loading the data to Qdrant, you need to have Qdrant running. You can run it using Docker:

```bash
docker run -d --name qdrant -p 6333:6333 qdrant/qdrant
```

Alternatively, you can use Qdrant Cloud: https://cloud.qdrant.io/

### Step 2: Update .env (if using cloud Qdrant)
If you're using Qdrant Cloud instead of the local instance, update your `.env` file:

```
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your-api-key-here
```

### Step 3: Load Data to Qdrant
Once Qdrant is running, load your book data:

```bash
cd backend
python load_to_qdrant.py
```

## Data Structure
The processed book data includes:

- URL of the original page
- Page title
- Content chunk (with intelligent chunking to preserve context)
- Chunk index and total chunks for the page
- Cohere embeddings (1024-dimensional vectors from embed-multilingual-v3.0 model)

## Verification
After loading, you can verify the data has been loaded correctly by:

1. Checking the Qdrant web UI at http://localhost:6333/dashboard (if running locally)
2. Using the following Python code to verify:
```python
from qdrant_client import QdrantClient
client = QdrantClient(host="localhost", port=6333)  # or your URL
count = client.count(collection_name="book_content")
print(f"Total vectors in collection: {count.count}")
```

## Usage with Your RAG System
Once the data is in Qdrant, your RAG system will automatically use Cohere embeddings when:
- `USE_COHERE_EMBEDDINGS=true` (already set in your .env)
- `COHERE_API_KEY` is properly configured (already set in your .env)

Your system will now be able to perform semantic search across your entire book content using Cohere-powered embeddings.

## Troubleshooting

### Qdrant Connection Issues
- Make sure Qdrant is running at the specified URL
- Check that the API key is correct (if using cloud Qdrant)
- Ensure the port is available (6333 by default)

### Embedding Generation Issues
- Verify that your COHERE_API_KEY is valid
- Check that you haven't exceeded your API quota

### Docker Installation
If you don't have Docker installed:
- Download from: https://www.docker.com/get-started/
- Or use Qdrant Cloud instead of local installation