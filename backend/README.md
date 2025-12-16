# RAG Chatbot Backend

This is the backend service for the Physical AI & Humanoid Robotics textbook RAG chatbot. It provides a FastAPI-based API that processes queries through vector search in Qdrant Cloud, using Google Gemini API for response generation.

## Features

- **RAG (Retrieval-Augmented Generation)**: Answers questions based on textbook content with zero hallucination tolerance
- **Selected Text Mode**: Answer questions based only on user-selected text
- **Source Citations**: Provides references to the original content
- **FastAPI**: Modern, fast web framework for building APIs with Python
- **Qdrant Vector Database**: Stores and searches text embeddings efficiently

## Prerequisites

- Python 3.11+
- Google Gemini API key
- Qdrant Cloud account and API key

## Setup

1. Clone the repository
2. Navigate to the `backend` directory
3. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
4. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
5. Set up environment variables:
   ```bash
   cp .env.example .env
   ```
   Then edit `.env` and add your API keys:
   - `GOOGLE_GEMINI_API_KEY`: Your Google Gemini API key
   - `QDRANT_API_KEY`: Your Qdrant Cloud API key
   - `QDRANT_URL`: Your Qdrant Cloud URL

## Usage

1. Start the development server:
   ```bash
   python main.py
   ```
   The API will be available at `http://localhost:8000`

2. The main endpoint is:
   - `POST /api/v1/chat/ask` - Ask a question about the textbook content

## API Endpoints

- `GET /` - Health check
- `POST /api/v1/chat/ask` - Ask a question
  - Request body: `{"question": "your question", "selected_text": "optional selected text"}`
  - Response: `{"answer": "answer", "sources": ["source1", "source2"], "selected_text_mode": true/false}`

## Data Pipeline

To populate the vector database with textbook content:

1. Run the content scraper:
   ```bash
   cd data_pipeline
   python scrape_content.py
   ```

2. Process and store the content:
   ```bash
   python embed_and_store.py
   ```

## Environment Variables

- `GOOGLE_GEMINI_API_KEY`: Google Gemini API key for text generation and embeddings
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `QDRANT_URL`: Qdrant Cloud URL
- `DOCUSAURUS_BASE_URL`: URL of the Docusaurus site to scrape (default: https://your-docusaurus-site.com)

## Architecture

The backend is organized as follows:

```
backend/
├── main.py                 # FastAPI application entry point
├── requirements.txt        # Python dependencies
├── .env                    # Environment variables
└── src/
    ├── models/
    │   └── query.py        # Request/response models
    ├── services/
    │   ├── rag_service.py     # Core RAG logic
    │   ├── embedding_service.py # Embedding generation and search
    │   └── qdrant_service.py  # Vector database operations
    └── api/
        └── v1/
            └── endpoints/
                └── chat.py   # Chat endpoint with /ask
```

## Development

1. Install dependencies in development mode
2. Run tests: `pytest tests/`
3. Format code: `black .` and `isort .`
4. Lint code: `flake8 .`

## Deployment

The backend can be deployed to cloud platforms like Render, Vercel, or any platform that supports Python applications. Make sure to set the environment variables in the deployment settings.