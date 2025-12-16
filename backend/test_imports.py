"""
Simple test to verify that the modules can be imported without errors.
This checks the code structure without actually running the embedding models.
"""
import sys
sys.path.append('.')

try:
    # Test importing the local embedding service
    from backend.src.services.local_embedding_service import LocalEmbeddingService
    print("✓ Successfully imported LocalEmbeddingService")

    # Test importing the main embedding service
    from backend.src.services.embedding_service import EmbeddingService
    print("✓ Successfully imported EmbeddingService")

    # Test creating instances (without actually calling the embedding models)
    print("\nTesting instance creation...")

    # Create a local embedding service instance
    local_service = LocalEmbeddingService.__new__(LocalEmbeddingService)  # Create without calling __init__
    local_service.dimension = 384  # Set default dimension
    local_service.tokenizer = None
    local_service.model = None
    local_service.device = "cpu"
    print("✓ Successfully created LocalEmbeddingService instance")

    # Test the embed_text method with the fallback case
    result = local_service.embed_text("test")
    print(f"✓ embed_text fallback works, returns vector of length: {len(result)}")

    print("\n✓ All imports and basic functionality tests passed!")
    print("\nTo run the full test with actual models, install the dependencies with:")
    print("  pip install -r backend/requirements.txt")
    print("\nThen run:")
    print("  python backend/test_local_embeddings.py")

except ImportError as e:
    print(f"✗ Import error: {e}")
except Exception as e:
    print(f"✗ Error: {e}")