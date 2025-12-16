from typing import List, Dict, Optional
from ..models.query import QueryRequest, RAGResult
from .qdrant_service import QdrantService
from .embedding_service import EmbeddingService
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Try to import dashscope for Qwen text generation
try:
    import dashscope
    DASHSCOPE_AVAILABLE = True
except ImportError:
    DASHSCOPE_AVAILABLE = False
    dashscope = None

class RAGService:
    def __init__(self):
        self.qdrant_service = QdrantService()
        self.embedding_service = EmbeddingService()

        # Check for Qwen API key first for text generation
        qwen_api_key = os.getenv("QWEN_API_KEY")
        use_qwen_generation = os.getenv("USE_QWEN_GENERATION", "false").lower() == "true"

        if use_qwen_generation and qwen_api_key and DASHSCOPE_AVAILABLE:
            dashscope.api_key = qwen_api_key
            self.use_qwen = True
            self.use_google = False
            self.model_type = "qwen"
            print("Using Qwen for text generation")
        else:
            # Initialize Google Generative AI
            google_api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
            if google_api_key:
                import google.generativeai as genai
                genai.configure(api_key=google_api_key)
                self.model = genai.GenerativeModel('gemini-pro')
                self.use_google = True
                self.use_qwen = False
                self.model_type = "google"
                print("Using Google Gemini for text generation")
            else:
                print("No Google or Qwen API key found, will return retrieved content directly")
                self.use_google = False
                self.use_qwen = False
                self.model_type = "none"
                self.model = None

    def process_query(self, request: QueryRequest) -> RAGResult:
        """
        Process a user query using RAG methodology
        """
        if request.selected_text:
            # Handle selected text mode - only use the provided selected text
            return self._process_selected_text_query(request)
        else:
            # Handle general question mode - search in the knowledge base
            return self._process_general_query(request)

    def _process_selected_text_query(self, request: QueryRequest) -> RAGResult:
        """
        Process query when user has selected specific text
        """
        # In selected text mode, we only use the provided selected text
        relevant_chunks = [{
            'text': request.selected_text,
            'url': 'selected_text',
            'title': 'User Selected Text'
        }]

        if self.use_qwen:
            # Generate answer based on selected text using Qwen
            prompt = f"""
            Answer the following question based ONLY on the provided selected text.
            Do not use any external knowledge or make assumptions beyond what's in the text.
            If the answer cannot be found in the provided text, say so explicitly.

            Selected Text: {request.selected_text}

            Question: {request.question}

            Answer:
            """

            try:
                response = dashscope.Generation.call(
                    model="qwen-max",  # Using Qwen's capable model
                    prompt=prompt,
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                if response.status_code == 200:
                    answer = response.output.text
                    # Prepend note about selected text mode
                    answer = f"Answering from selected text only:\n\n{answer}"
                else:
                    # Fallback if Qwen API fails
                    answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Qwen API error - {response.code}: {response.message}"
            except Exception as e:
                # Fallback if Qwen generation fails
                answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Qwen generation error - {str(e)}"
        elif self.use_google:
            # Generate answer based on selected text using Google
            prompt = f"""
            Answer the following question based ONLY on the provided selected text.
            Do not use any external knowledge or make assumptions beyond what's in the text.
            If the answer cannot be found in the provided text, say so explicitly.

            Selected Text: {request.selected_text}

            Question: {request.question}

            Answer:
            """

            try:
                response = self.model.generate_content(
                    prompt,
                    generation_config={
                        "temperature": 0.1,  # Low temperature for factual accuracy
                        "max_output_tokens": 1000
                    }
                )

                answer = response.text

                # Prepend note about selected text mode
                answer = f"Answering from selected text only:\n\n{answer}"
            except Exception as e:
                # Fallback if Google generation fails
                answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Google generation error - {str(e)}"
        else:
            # Without Google or Qwen, just return the selected text with a note
            answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Without Google or Qwen API key, full answer generation is not available."

        return RAGResult(
            answer=answer,
            relevant_chunks=relevant_chunks,
            sources=["Selected Text"],
            query_embedding=None
        )

    def _process_general_query(self, request: QueryRequest) -> RAGResult:
        """
        Process general query by searching in the knowledge base
        """
        # Generate embedding for the query
        query_embedding = self.embedding_service.embed_text(request.question)

        # Search for relevant chunks in the vector database
        relevant_chunks = self.qdrant_service.search_similar(
            query_embedding,
            limit=request.context_window
        )

        if not relevant_chunks:
            # If no relevant chunks found, return appropriate response
            return RAGResult(
                answer="I couldn't find relevant information in the textbook to answer your question.",
                relevant_chunks=[],
                sources=[],
                query_embedding=query_embedding
            )

        # Prepare context from relevant chunks
        context_texts = [chunk['text'] for chunk in relevant_chunks]
        context = "\n\n".join(context_texts)

        if self.use_qwen:
            # Generate prompt for Qwen
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the following question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make assumptions beyond what's in the context.
            If the answer cannot be found in the context, say so explicitly.

            Context:
            {context}

            Question: {request.question}

            Answer (include source citations if possible):
            """

            try:
                response = dashscope.Generation.call(
                    model="qwen-max",  # Using Qwen's capable model
                    prompt=prompt,
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                if response.status_code == 200:
                    answer = response.output.text
                else:
                    # Fallback if Qwen API fails
                    answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Qwen API error - {response.code}: {response.message}. The relevant information from the textbook is shown above."
            except Exception as e:
                # Fallback if Qwen generation fails
                answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Qwen generation error - {str(e)}. The relevant information from the textbook is shown above."
        elif self.use_google:
            # Generate prompt for the LLM
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the following question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make assumptions beyond what's in the context.
            If the answer cannot be found in the context, say so explicitly.

            Context:
            {context}

            Question: {request.question}

            Answer (include source citations if possible):
            """

            try:
                response = self.model.generate_content(
                    prompt,
                    generation_config={
                        "temperature": 0.1,  # Low temperature for factual accuracy
                        "max_output_tokens": 1000
                    }
                )

                answer = response.text
            except Exception as e:
                # Fallback if Google generation fails
                answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Google generation error - {str(e)}. The relevant information from the textbook is shown above."
        else:
            # Without Google or Qwen, return the retrieved context with the question
            answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Without Google or Qwen API key, full answer generation is not available. The relevant information from the textbook is shown above."

        # Extract sources from relevant chunks
        sources = []
        for chunk in relevant_chunks:
            source = chunk.get('url', 'Unknown source')
            if source and source != 'Unknown source':
                sources.append(source)

        return RAGResult(
            answer=answer,
            relevant_chunks=relevant_chunks,
            sources=sources,
            query_embedding=query_embedding
        )

    def get_answer_with_sources(self, request: QueryRequest) -> RAGResult:
        """
        Main method to get answer with proper source citations
        """
        result = self.process_query(request)

        # Add source citations to the answer if sources exist
        if result.sources:
            result.answer += f"\n\nSources: {', '.join(result.sources[:3])}"  # Limit to first 3 sources

        return result