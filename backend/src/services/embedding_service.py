import os
import google.generativeai as genai
from typing import List, Optional
from dotenv import load_dotenv
from .local_embedding_service import LocalEmbeddingService

# Try to import dashscope for Qwen embeddings
try:
    import dashscope
    DASHSCOPE_AVAILABLE = True
except ImportError:
    DASHSCOPE_AVAILABLE = False
    dashscope = None

# Load environment variables
load_dotenv()


class EmbeddingService:
    def __init__(self):
        # Check if we should use Qwen embeddings
        use_qwen_embeddings = os.getenv("USE_QWEN_EMBEDDINGS", "false").lower() == "true"

        if use_qwen_embeddings:
            print("Using Qwen embeddings")
            self.use_qwen = True
            self.use_local = False
            # Initialize Qwen/DashScope
            api_key = os.getenv("QWEN_API_KEY")
            if not api_key:
                print("Warning: QWEN_API_KEY not found, falling back to Google embeddings")
                self.use_qwen = False
                use_qwen_embeddings = False
            else:
                if DASHSCOPE_AVAILABLE:
                    dashscope.api_key = api_key
                    self.embedding_model = "text-embedding-v2"  # Qwen's embedding model
                    # Qwen's embedding model produces 1536-dimensional vectors
                    self.dimension = 1536
                else:
                    print("Warning: dashscope not installed, falling back to Google embeddings")
                    self.use_qwen = False
                    use_qwen_embeddings = False

        # Check if we should use local embeddings
        use_local_embeddings = os.getenv("USE_LOCAL_EMBEDDINGS", "false").lower() == "true" and not use_qwen_embeddings

        if use_local_embeddings:
            print("Using local embeddings")
            self.use_local = True
            self.local_service = LocalEmbeddingService()
            # Set dimension based on local model
            self.dimension = self.local_service.dimension
        elif not use_qwen_embeddings:  # Only initialize Google if not using Qwen
            print("Using Google embeddings")
            self.use_local = False
            # Initialize Google Generative AI
            api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
            if not api_key:
                print("Warning: GOOGLE_GEMINI_API_KEY not found, falling back to local embeddings")
                self.use_local = True
                self.local_service = LocalEmbeddingService()
                self.dimension = self.local_service.dimension
            else:
                genai.configure(api_key=api_key)
                self.embedding_model = "models/embedding-001"
                # Google's embedding-001 model produces 768-dimensional vectors
                self.dimension = 768

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        if hasattr(self, 'use_qwen') and self.use_qwen:
            # Use Qwen embeddings
            if not DASHSCOPE_AVAILABLE:
                print("dashscope not available, falling back to local embeddings")
                self.use_qwen = False
                self.use_local = True
                if not hasattr(self, 'local_service'):
                    self.local_service = LocalEmbeddingService()
                    self.dimension = self.local_service.dimension
                return self.local_service.embed_text(text)

            try:
                response = dashscope.TextEmbedding.call(
                    model=self.embedding_model,
                    input=text,  # Pass the text directly, not as a list
                    text_type='query'  # Use 'query' for retrieval queries, 'document' for documents to be stored
                )

                if response.status_code == 200:
                    return response.output['embeddings'][0]['embedding']  # Return the first embedding
                else:
                    print(f"Error from Qwen API: {response.code} - {response.message}")
                    # Fallback to local embeddings
                    self.use_local = True
                    if not hasattr(self, 'local_service'):
                        self.local_service = LocalEmbeddingService()
                        self.dimension = self.local_service.dimension
                    return self.local_service.embed_text(text)
            except Exception as e:
                print(f"Error generating Qwen embedding: {e}")
                # Fallback to local embeddings
                self.use_local = True
                if not hasattr(self, 'local_service'):
                    self.local_service = LocalEmbeddingService()
                    self.dimension = self.local_service.dimension
                return self.local_service.embed_text(text)
        elif self.use_local:
            return self.local_service.embed_text(text)
        else:
            # Use Google embeddings
            try:
                response = genai.embed_content(
                    model=self.embedding_model,
                    content=[text],  # API expects a list
                    task_type="RETRIEVAL_QUERY"  # Use "RETRIEVAL_DOCUMENT" for documents to be stored
                )
                return response['embedding'][0]  # Return the first (and only) embedding
            except Exception as e:
                print(f"Error generating Google embedding: {e}")
                # Check if it's a quota exceeded error
                if "quota" in str(e).lower() or "429" in str(e):
                    print("Quota exceeded for Google Generative AI embeddings. Switching to local embeddings.")
                    # Switch to local embeddings and try again
                    self.use_local = True
                    if not hasattr(self, 'local_service'):
                        self.local_service = LocalEmbeddingService()
                        self.dimension = self.local_service.dimension
                    return self.local_service.embed_text(text)
                else:
                    # Return a zero vector in case of error
                    return [0.0] * self.dimension

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        """
        # For better performance, we could implement batch processing for each provider
        # but for now, we'll call embed_text for each text individually
        # (This allows proper fallback handling for each individual text)
        embeddings = []
        for text in texts:
            embedding = self.embed_text(text)
            embeddings.append(embedding)
        return embeddings

    def compute_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Compute cosine similarity between two embeddings
        """
        if hasattr(self, 'use_qwen') and self.use_qwen:
            # Use cosine similarity calculation for Qwen embeddings (same as general calculation)
            # Calculate dot product
            dot_product = sum(a * b for a, b in zip(embedding1, embedding2))

            # Calculate magnitudes
            magnitude1 = sum(a * a for a in embedding1) ** 0.5
            magnitude2 = sum(a * a for a in embedding2) ** 0.5

            # Calculate cosine similarity
            if magnitude1 == 0 or magnitude2 == 0:
                return 0.0

            return dot_product / (magnitude1 * magnitude2)
        elif self.use_local:
            return self.local_service.compute_similarity(embedding1, embedding2)
        else:
            # Use Google embeddings similarity calculation (same as general calculation)
            # Calculate dot product
            dot_product = sum(a * b for a, b in zip(embedding1, embedding2))

            # Calculate magnitudes
            magnitude1 = sum(a * a for a in embedding1) ** 0.5
            magnitude2 = sum(a * a for a in embedding2) ** 0.5

            # Calculate cosine similarity
            if magnitude1 == 0 or magnitude2 == 0:
                return 0.0

            return dot_product / (magnitude1 * magnitude2)