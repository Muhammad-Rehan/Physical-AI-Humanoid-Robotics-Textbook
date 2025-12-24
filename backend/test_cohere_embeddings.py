#!/usr/bin/env python3
"""
Test script to verify Cohere embedding functionality
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.embedding_service import EmbeddingService

def test_embedding_service():
    print("Testing EmbeddingService with Cohere configurations...")

    # Test 1: Cohere embeddings (if API key is available)
    print("\n1. Testing with Cohere embeddings (if configured):")
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    
    # Only test Cohere if API key is set
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if cohere_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
        test_text = "This is a test sentence for Cohere embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
    else:
        print("   Skipping Cohere test - no API key found")
        # Test the fallback behavior
        print("   Testing Cohere fallback without API key...")
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
        print(f"   Using Qwen: {getattr(service, 'use_qwen', False)}")
        print(f"   Using local: {getattr(service, 'use_local', False)}")
        test_text = "This is a test sentence for embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")

    # Test 2: Cohere with different models
    print("\n2. Testing with Cohere multilingual-light-v3.0 model (if configured):")
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    os.environ["COHERE_EMBED_MODEL"] = "embed-multilingual-light-v3.0"
    
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if cohere_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
        test_text = "This is a test sentence for Cohere light embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
    else:
        print("   Skipping Cohere light model test - no API key found")

    # Test 3: Cohere with English model
    print("\n3. Testing with Cohere embed-english-v3.0 model (if configured):")
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    os.environ["COHERE_EMBED_MODEL"] = "embed-english-v3.0"
    
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if cohere_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
        test_text = "This is a test sentence for Cohere English embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
    else:
        print("   Skipping Cohere English model test - no API key found")

if __name__ == "__main__":
    test_embedding_service()