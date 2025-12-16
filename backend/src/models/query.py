from pydantic import BaseModel
from typing import Optional, List


class QueryRequest(BaseModel):
    """
    Request model for the chat endpoint
    """
    question: str
    selected_text: Optional[str] = None
    context_window: Optional[int] = 3  # Number of relevant chunks to retrieve


class QueryResponse(BaseModel):
    """
    Response model for the chat endpoint
    """
    answer: str
    sources: List[str]  # List of source references
    selected_text_mode: bool = False  # Whether response is based only on selected text
    confidence: Optional[float] = None  # Confidence score between 0 and 1


class RAGResult(BaseModel):
    """
    Model for RAG processing results
    """
    answer: str
    relevant_chunks: List[dict]  # List of relevant text chunks with metadata
    sources: List[str]
    query_embedding: Optional[List[float]] = None