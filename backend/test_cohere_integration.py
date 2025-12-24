#!/usr/bin/env python3
"""
Quick test script to verify Cohere embedding functionality
"""
import os
import sys
import importlib.util

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

def test_cohere_import():
    """Test if Cohere module can be imported properly"""
    try:
        import cohere
        print("+ Cohere module imported successfully")
        return True
    except ImportError as e:
        print(f"- Failed to import Cohere: {e}")
        return False

def test_embedding_service_creation():
    """Test creating the embedding service with Cohere configuration"""
    from src.services.embedding_service import EmbeddingService

    # Save original env vars
    original_cohere_embed = os.environ.get("USE_COHERE_EMBEDDINGS")
    original_qwen_embed = os.environ.get("USE_QWEN_EMBEDDINGS")
    original_local_embed = os.environ.get("USE_LOCAL_EMBEDDINGS")

    # Configure to use Cohere (but will fall back if no API key)
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"

    try:
        service = EmbeddingService()
        print(f"+ EmbeddingService created successfully")
        print(f"  - Uses Cohere: {getattr(service, 'use_cohere', False)}")
        print(f"  - Uses Qwen: {getattr(service, 'use_qwen', False)}")
        print(f"  - Uses Local: {getattr(service, 'use_local', False)}")
        print(f"  - Dimension: {service.dimension}")
        return True
    except Exception as e:
        print(f"- Failed to create EmbeddingService: {e}")
        return False
    finally:
        # Restore original env vars
        if original_cohere_embed is not None:
            os.environ["USE_COHERE_EMBEDDINGS"] = original_cohere_embed
        else:
            os.environ.pop("USE_COHERE_EMBEDDINGS", None)

        if original_qwen_embed is not None:
            os.environ["USE_QWEN_EMBEDDINGS"] = original_qwen_embed
        else:
            os.environ.pop("USE_QWEN_EMBEDDINGS", None)

        if original_local_embed is not None:
            os.environ["USE_LOCAL_EMBEDDINGS"] = original_local_embed
        else:
            os.environ.pop("USE_LOCAL_EMBEDDINGS", None)

def test_embedding_generation():
    """Test embedding generation with the service"""
    from src.services.embedding_service import EmbeddingService

    # Save original env vars
    original_cohere_embed = os.environ.get("USE_COHERE_EMBEDDINGS")
    original_qwen_embed = os.environ.get("USE_QWEN_EMBEDDINGS")
    original_local_embed = os.environ.get("USE_LOCAL_EMBEDDINGS")

    # Configure to use Cohere (but will fall back if no API key)
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"

    try:
        service = EmbeddingService()
        test_text = "This is a test sentence for embedding."

        embedding = service.embed_text(test_text)
        print(f"+ Embedding generated successfully")
        print(f"  - Embedding length: {len(embedding)}")
        print(f"  - First 5 values: {embedding[:5]}")
        return True
    except Exception as e:
        print(f"- Failed to generate embedding: {e}")
        return False
    finally:
        # Restore original env vars
        if original_cohere_embed is not None:
            os.environ["USE_COHERE_EMBEDDINGS"] = original_cohere_embed
        else:
            os.environ.pop("USE_COHERE_EMBEDDINGS", None)

        if original_qwen_embed is not None:
            os.environ["USE_QWEN_EMBEDDINGS"] = original_qwen_embed
        else:
            os.environ.pop("USE_QWEN_EMBEDDINGS", None)

        if original_local_embed is not None:
            os.environ["USE_LOCAL_EMBEDDINGS"] = original_local_embed
        else:
            os.environ.pop("USE_LOCAL_EMBEDDINGS", None)

def main():
    print("Running Cohere integration tests...\n")

    tests = [
        ("Cohere Import Test", test_cohere_import),
        ("Embedding Service Creation Test", test_embedding_service_creation),
        ("Embedding Generation Test", test_embedding_generation),
    ]

    passed = 0
    total = len(tests)

    for test_name, test_func in tests:
        print(f"{test_name}:")
        if test_func():
            passed += 1
        print()

    print(f"Tests passed: {passed}/{total}")

    if passed == total:
        print(":) All tests passed! Cohere integration is working correctly.")
    else:
        print(":( Some tests failed. Please check the output above for details.")

if __name__ == "__main__":
    main()