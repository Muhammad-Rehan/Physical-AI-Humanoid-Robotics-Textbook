#!/usr/bin/env python3
"""
Test script to verify Qwen embedding functionality
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.embedding_service import EmbeddingService

def test_embedding_service():
    print("Testing EmbeddingService with different configurations...")

    # Test 1: Local embeddings (current default)
    print("\n1. Testing with local embeddings (default):")
    os.environ["USE_LOCAL_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ.pop("GOOGLE_GEMINI_API_KEY", None)  # Remove Google API key if present

    service = EmbeddingService()
    print(f"   Service dimension: {service.dimension}")
    print(f"   Using local: {getattr(service, 'use_local', False)}")

    test_text = "This is a test sentence for embedding."
    embedding = service.embed_text(test_text)
    print(f"   Embedding length: {len(embedding)}")
    print(f"   First 5 values: {embedding[:5]}")

    # Test 2: Google embeddings (if API key is available)
    print("\n2. Testing with Google embeddings (if configured):")
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"

    # Only test Google if API key is set
    google_api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
    if google_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
    else:
        print("   Skipping Google test - no API key found")

    # Test 3: Qwen embeddings (if API key is available)
    print("\n3. Testing with Qwen embeddings (if configured):")
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    os.environ["USE_QWEN_EMBEDDINGS"] = "true"

    # Only test Qwen if API key is set
    qwen_api_key = os.getenv("QWEN_API_KEY")
    if qwen_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Qwen: {getattr(service, 'use_qwen', False)}")
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
    else:
        print("   Skipping Qwen test - no API key found")
        # Test the fallback behavior
        print("   Testing Qwen fallback without API key...")
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Qwen: {getattr(service, 'use_qwen', False)}")
        print(f"   Using local: {getattr(service, 'use_local', False)}")
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")

if __name__ == "__main__":
    test_embedding_service()