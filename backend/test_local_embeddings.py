import os
import sys

# Add the backend/src directory to the path so we can import modules
sys.path.append('.')

from backend.src.services.local_embedding_service import LocalEmbeddingService
from backend.src.services.embedding_service import EmbeddingService

def test_local_embeddings():
    print("Testing Local Embedding Service...")

    # Test the local embedding service directly
    local_service = LocalEmbeddingService()

    # Test single text embedding
    text = "This is a sample text for embedding."
    embedding = local_service.embed_text(text)
    print(f"Local embedding length: {len(embedding)}")
    print(f"First 5 dimensions: {embedding[:5]}")

    # Test multiple texts
    texts = [
        "Machine learning is a subset of artificial intelligence.",
        "Deep learning uses neural networks with multiple layers.",
        "Natural language processing helps computers understand text."
    ]
    embeddings = local_service.embed_documents(texts)
    print(f"Generated {len(embeddings)} embeddings")

    # Test similarity calculation
    similarity = local_service.compute_similarity(embeddings[0], embeddings[1])
    print(f"Similarity between first two texts: {similarity:.4f}")

    print("\nTesting Flexible Embedding Service with local embeddings enabled...")

    # Set environment variable to use local embeddings
    os.environ["USE_LOCAL_EMBEDDINGS"] = "true"

    # Create embedding service (should now use local)
    flex_service = EmbeddingService()

    # Test embedding with the flexible service
    text2 = "Artificial intelligence is transforming technology."
    embedding2 = flex_service.embed_text(text2)
    print(f"Flexible service embedding length: {len(embedding2)}")
    print(f"First 5 dimensions: {embedding2[:5]}")

    # Test similarity with flexible service
    similarity2 = flex_service.compute_similarity(embedding2, embeddings[0])
    print(f"Similarity with flexible service: {similarity2:.4f}")

    print("\nAll tests completed successfully!")

if __name__ == "__main__":
    test_local_embeddings()