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

# Try to import cohere for Cohere embeddings
try:
    import cohere
    COHERE_AVAILABLE = True
except ImportError:
    COHERE_AVAILABLE = False
    cohere = None

# Load environment variables
load_dotenv()


class EmbeddingService:
    def __init__(self):
        # Check if we should use Cohere embeddings
        use_cohere_embeddings = os.getenv("USE_COHERE_EMBEDDINGS", "false").lower() == "true"

        if use_cohere_embeddings:
            print("Using Cohere embeddings")
            self.use_cohere = True
            self.use_qwen = False
            self.use_local = False
            # Initialize Cohere
            api_key = os.getenv("COHERE_API_KEY")
            if not api_key:
                print("Warning: COHERE_API_KEY not found, falling back to Qwen embeddings")
                self.use_cohere = False
                use_cohere_embeddings = False
            else:
                if COHERE_AVAILABLE:
                    self.cohere_client = cohere.Client(api_key)
                    # Get the embedding model from environment, default to embed-multilingual-v3.0
                    self.embedding_model = os.getenv("COHERE_EMBED_MODEL", "embed-multilingual-v3.0")

                    # Set dimension based on the selected model
                    if self.embedding_model in ['embed-multilingual-v3.0', 'embed-english-v3.0']:
                        self.dimension = 1024
                    elif self.embedding_model in ['embed-english-light-v3.0', 'embed-multilingual-light-v3.0']:
                        self.dimension = 384
                    else:
                        # Default to embed-multilingual-v3.0 if an unknown model is specified
                        self.embedding_model = "embed-multilingual-v3.0"
                        self.dimension = 1024
                else:
                    print("Warning: cohere not installed, falling back to Qwen embeddings")
                    self.use_cohere = False
                    use_cohere_embeddings = False

        # Check if we should use Qwen embeddings
        use_qwen_embeddings = os.getenv("USE_QWEN_EMBEDDINGS", "false").lower() == "true" and not use_cohere_embeddings

        if use_qwen_embeddings:
            print("Using Qwen embeddings")
            self.use_qwen = True
            self.use_cohere = False
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
        use_local_embeddings = os.getenv("USE_LOCAL_EMBEDDINGS", "false").lower() == "true" and not use_cohere_embeddings and not use_qwen_embeddings

        if use_local_embeddings:
            print("Using local embeddings")
            self.use_local = True
            self.use_cohere = False
            self.use_qwen = False
            self.local_service = LocalEmbeddingService()
            # Set dimension based on local model
            self.dimension = self.local_service.dimension
        elif not use_cohere_embeddings and not use_qwen_embeddings:  # Only initialize Google if not using Cohere or Qwen
            print("Using Google embeddings")
            self.use_local = False
            self.use_cohere = False
            self.use_qwen = False
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
        if hasattr(self, 'use_cohere') and self.use_cohere:
            # Use Cohere embeddings
            if not COHERE_AVAILABLE:
                print("cohere not available, falling back to Qwen embeddings")
                self.use_cohere = False
                self.use_qwen = True

                # Re-initialize Qwen if needed
                api_key = os.getenv("QWEN_API_KEY")
                if not api_key:
                    print("Warning: QWEN_API_KEY not found, falling back to Google embeddings")
                    self.use_qwen = False
                    self.use_local = True
                    if not hasattr(self, 'local_service'):
                        self.local_service = LocalEmbeddingService()
                        self.dimension = self.local_service.dimension
                    return self.local_service.embed_text(text)

                if DASHSCOPE_AVAILABLE:
                    dashscope.api_key = api_key
                    self.embedding_model = "text-embedding-v2"  # Qwen's embedding model
                    # Qwen's embedding model produces 1536-dimensional vectors
                    self.dimension = 1536
                else:
                    print("Warning: dashscope not installed, falling back to Google embeddings")
                    self.use_qwen = False
                    self.use_local = True
                    if not hasattr(self, 'local_service'):
                        self.local_service = LocalEmbeddingService()
                        self.dimension = self.local_service.dimension
                    return self.local_service.embed_text(text)

            try:
                response = self.cohere_client.embed(
                    texts=[text],
                    model=self.embedding_model,
                    input_type="search_query"  # Required for Cohere v3 models
                )

                return response.embeddings[0]  # Return the first embedding
            except Exception as e:
                print(f"Error generating Cohere embedding: {e}")
                # Fallback to Qwen embeddings
                self.use_cohere = False
                self.use_qwen = True

                # Re-initialize Qwen if needed
                api_key = os.getenv("QWEN_API_KEY")
                if api_key and DASHSCOPE_AVAILABLE:
                    dashscope.api_key = api_key
                    self.embedding_model = "text-embedding-v2"  # Qwen's embedding model
                    self.dimension = 1536
                else:
                    print("Falling back further to Google or local embeddings")
                    self.use_qwen = False
                    self.use_local = True
                    if not hasattr(self, 'local_service'):
                        self.local_service = LocalEmbeddingService()
                        self.dimension = self.local_service.dimension
                    return self.local_service.embed_text(text)

                # Call embed_text again with the new configuration
                return self.embed_text(text)

        elif hasattr(self, 'use_qwen') and self.use_qwen:
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
        if hasattr(self, 'use_cohere') and self.use_cohere:
            # Use Cohere embeddings with batch processing for better performance
            if not COHERE_AVAILABLE:
                print("cohere not available, falling back to Qwen embeddings")
                self.use_cohere = False
                self.use_qwen = True

                # Re-initialize Qwen if needed
                api_key = os.getenv("QWEN_API_KEY")
                if api_key and DASHSCOPE_AVAILABLE:
                    dashscope.api_key = api_key
                    self.embedding_model = "text-embedding-v2"  # Qwen's embedding model
                    self.dimension = 1536
                else:
                    print("Falling back further to Google or local embeddings")
                    self.use_qwen = False
                    self.use_local = True
                    if not hasattr(self, 'local_service'):
                        self.local_service = LocalEmbeddingService()
                        self.dimension = self.local_service.dimension
                    return [self.local_service.embed_text(text) for text in texts]

                # Call embed_documents again with the new configuration
                return self.embed_documents(texts)

            try:
                response = self.cohere_client.embed(
                    texts=texts,
                    model=self.embedding_model,
                    input_type="search_document"  # For documents, use different input type than for queries
                )

                return response.embeddings  # Return all embeddings
            except Exception as e:
                print(f"Error generating Cohere embeddings: {e}")
                # Fallback to Qwen embeddings
                self.use_cohere = False
                self.use_qwen = True

                # Re-initialize Qwen if needed
                api_key = os.getenv("QWEN_API_KEY")
                if api_key and DASHSCOPE_AVAILABLE:
                    dashscope.api_key = api_key
                    self.embedding_model = "text-embedding-v2"  # Qwen's embedding model
                    self.dimension = 1536
                else:
                    print("Falling back further to Google or local embeddings")
                    self.use_qwen = False
                    self.use_local = True
                    if not hasattr(self, 'local_service'):
                        self.local_service = LocalEmbeddingService()
                        self.dimension = self.local_service.dimension

                    # Call embed_documents again with the new configuration
                    return [self.local_service.embed_text(text) for text in texts]

                # Call embed_documents again with the new configuration
                return self.embed_documents(texts)
        else:
            # For other providers (Qwen, Google, local), we'll call embed_text for each text individually
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
        if hasattr(self, 'use_cohere') and self.use_cohere:
            # Use cosine similarity calculation for Cohere embeddings (same as general calculation)
            # Calculate dot product
            dot_product = sum(a * b for a, b in zip(embedding1, embedding2))

            # Calculate magnitudes
            magnitude1 = sum(a * a for a in embedding1) ** 0.5
            magnitude2 = sum(a * a for a in embedding2) ** 0.5

            # Calculate cosine similarity
            if magnitude1 == 0 or magnitude2 == 0:
                return 0.0

            return dot_product / (magnitude1 * magnitude2)
        elif hasattr(self, 'use_qwen') and self.use_qwen:
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