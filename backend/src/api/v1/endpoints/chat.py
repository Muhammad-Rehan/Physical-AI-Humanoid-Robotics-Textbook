from fastapi import APIRouter, HTTPException
from src.models.query import QueryRequest, QueryResponse
from src.services.rag_service import RAGService

router = APIRouter(prefix="/chat", tags=["chat"])

# Initialize the RAG service
rag_service = RAGService()

@router.post("/ask", response_model=QueryResponse)
async def ask_question(request: QueryRequest):
    """
    Endpoint to ask a question and get an answer based on the textbook content
    """
    try:
        # Process the query using the RAG service
        rag_result = rag_service.get_answer_with_sources(request)

        # Determine if this is in selected text mode
        selected_text_mode = request.selected_text is not None and len(request.selected_text.strip()) > 0

        # Create the response
        response = QueryResponse(
            answer=rag_result.answer,
            sources=rag_result.sources,
            selected_text_mode=selected_text_mode,
            confidence=0.8  # Placeholder confidence value
        )

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")

@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "RAG Chatbot API"}

@router.get("/stats")
async def get_stats():
    """
    Get statistics about the RAG system
    """
    try:
        # This would connect to Qdrant to get collection stats
        # For now, return placeholder values
        return {
            "status": "connected",
            "documents_indexed": 0,  # This would come from Qdrant
            "last_updated": "2025-12-10"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error getting stats: {str(e)}")