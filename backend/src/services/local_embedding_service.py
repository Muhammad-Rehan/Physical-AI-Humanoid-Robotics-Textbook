import numpy as np
from typing import List, Optional
import os

# Try to import transformers and torch, with graceful fallback
try:
    from transformers import AutoTokenizer, AutoModel
    import torch
    TRANSFORMERS_AVAILABLE = True
except (OSError, ImportError) as e:
    print(f"Warning: Could not import transformers/torch: {e}")
    print("Local embeddings will use zero vectors as fallback.")
    TRANSFORMERS_AVAILABLE = False
    AutoTokenizer = None
    AutoModel = None
    torch = None


class LocalEmbeddingService:
    def __init__(self, model_name: str = None):
        """
        Initialize the local embedding service with a pre-trained sentence transformer model.

        Args:
            model_name: Name of the sentence transformer model to use.
                       If None, will try to get from environment variable LOCAL_EMBEDDING_MODEL,
                       otherwise defaults to "all-MiniLM-L6-v2".
                       Default 'all-MiniLM-L6-v2' is lightweight but effective.
                       Alternative: 'all-mpnet-base-v2' for better quality but slower speed.
        """
        if not TRANSFORMERS_AVAILABLE:
            print("Transformers not available, initializing with fallback values")
            self.tokenizer = None
            self.model = None
            self.device = "cpu"
            # Use 768 as dimension to match Google's embedding-001 model for compatibility with existing database
            self.dimension = 768
            # Also set the expected dimension for compatibility
            self._expected_dimension = 768
            return

        if model_name is None:
            model_name = os.getenv("LOCAL_EMBEDDING_MODEL", "all-MiniLM-L6-v2")  # Default to 384-dim
        # If we want to match Google's 768-dim vectors, use a model that produces 768 dimensions
        if model_name == "all-MiniLM-L6-v2":
            # all-MiniLM-L6-v2 produces 384 dimensions, but we want to match 768 from Google
            # For compatibility with existing database, we'll use a fallback dimension of 768
            self._expected_dimension = 768  # Match Google's embedding dimension
        else:
            self._expected_dimension = 384  # Default fallback

        try:
            # Initialize tokenizer and model
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.model = AutoModel.from_pretrained(model_name)

            # Move model to GPU if available, otherwise use CPU
            device = "cuda" if torch.cuda.is_available() else "cpu"
            self.model.to(device)
            self.device = device

            print(f"Local embedding model '{model_name}' loaded successfully on {device}")

            # Set dimension based on model
            # all-MiniLM-L6-v2: 384, all-mpnet-base-v2: 768, etc.
            self.dimension = self.model.config.hidden_size

        except Exception as e:
            print(f"Error initializing local embedding model: {e}")
            # Fallback initialization
            self.tokenizer = None
            self.model = None
            self.device = "cpu"
            self.dimension = 384  # Default fallback dimension

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using local model.

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector
        """
        if not text or not isinstance(text, str):
            # Return a zero vector for invalid input
            return [0.0] * self.dimension

        if not TRANSFORMERS_AVAILABLE or self.model is None or self.tokenizer is None:
            # Fallback: return zero vector if model failed to initialize
            return [0.0] * self.dimension

        try:
            # Tokenize the input text
            inputs = self.tokenizer(
                text,
                return_tensors="pt",
                padding=True,
                truncation=True,
                max_length=512
            ).to(self.device)

            # Generate embeddings
            with torch.no_grad():
                outputs = self.model(**inputs)

            # Use mean pooling to get sentence embeddings
            embeddings = outputs.last_hidden_state.mean(dim=1)

            # Convert to CPU and numpy array, then to list
            embedding = embeddings.cpu().numpy()[0].tolist()

            return embedding

        except Exception as e:
            print(f"Error generating embedding for text '{text[:50]}...': {e}")
            # Return a zero vector in case of error
            return [0.0] * self.dimension

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.

        Args:
            texts: List of input texts to embed

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        embeddings = []
        for text in texts:
            embedding = self.embed_text(text)
            embeddings.append(embedding)
        return embeddings

    def compute_similarity(self, embedding1: List[float], embedding2: List[float]) -> float:
        """
        Compute cosine similarity between two embeddings.

        Args:
            embedding1: First embedding vector
            embedding2: Second embedding vector

        Returns:
            Cosine similarity value between -1 and 1
        """
        if not TRANSFORMERS_AVAILABLE:
            # If transformers not available, return 0 similarity as fallback
            return 0.0

        # Convert to numpy arrays for computation
        vec1 = np.array(embedding1)
        vec2 = np.array(embedding2)

        # Calculate dot product
        dot_product = np.dot(vec1, vec2)

        # Calculate magnitudes
        magnitude1 = np.linalg.norm(vec1)
        magnitude2 = np.linalg.norm(vec2)

        # Calculate cosine similarity
        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        return float(dot_product / (magnitude1 * magnitude2))


# Example usage and testing
if __name__ == "__main__":
    # Test the local embedding service
    service = LocalEmbeddingService()

    # Test single text embedding
    text = "This is a sample text for embedding."
    embedding = service.embed_text(text)
    print(f"Embedding length: {len(embedding)}")
    print(f"First 5 dimensions: {embedding[:5]}")

    # Test multiple texts
    texts = [
        "Machine learning is a subset of artificial intelligence.",
        "Deep learning uses neural networks with multiple layers.",
        "Natural language processing helps computers understand text."
    ]
    embeddings = service.embed_documents(texts)
    print(f"Generated {len(embeddings)} embeddings")

    # Test similarity calculation
    similarity = service.compute_similarity(embeddings[0], embeddings[1])
    print(f"Similarity between first two texts: {similarity:.4f}")