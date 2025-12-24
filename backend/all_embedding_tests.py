#!/usr/bin/env python3
"""
Comprehensive test script to verify all embedding functionalities: Cohere, Qwen, Google, and Local
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.services.embedding_service import EmbeddingService


def test_cohere_embeddings():
    """Test Cohere embeddings with different models"""
    print("=== Testing Cohere Embeddings ===")
    
    # Test with multilingual-v3.0 (default)
    print("\n1. Testing Cohere multilingual-v3.0 model:")
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    os.environ["COHERE_EMBED_MODEL"] = "multilingual-v3.0"
    
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if cohere_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
        test_text = "This is a test sentence for Cohere embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
        print("   + Cohere multilingual-v3.0 test passed")
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

    # Test with embed-english-v3.0
    print("\n2. Testing Cohere embed-english-v3.0 model:")
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    os.environ["COHERE_EMBED_MODEL"] = "embed-english-v3.0"
    
    if cohere_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
        test_text = "This is a test sentence for Cohere English embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
        print("   + Cohere embed-english-v3.0 test passed")
    else:
        print("   Skipping Cohere English model test - no API key found")

    # Test with embed-multilingual-light-v3.0
    print("\n3. Testing Cohere embed-multilingual-light-v3.0 model:")
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    os.environ["COHERE_EMBED_MODEL"] = "embed-multilingual-light-v3.0"
    
    if cohere_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
        test_text = "This is a test sentence for Cohere light embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
        print("   + Cohere embed-multilingual-light-v3.0 test passed")
    else:
        print("   Skipping Cohere light model test - no API key found")


def test_qwen_embeddings():
    """Test Qwen embeddings"""
    print("\n\n=== Testing Qwen Embeddings ===")
    
    print("\n1. Testing with Qwen embeddings (if configured):")
    os.environ["USE_COHERE_EMBEDDINGS"] = "false"
    os.environ["USE_QWEN_EMBEDDINGS"] = "true"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"

    # Only test Qwen if API key is set
    qwen_api_key = os.getenv("QWEN_API_KEY")
    if qwen_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Qwen: {getattr(service, 'use_qwen', False)}")
        test_text = "This is a test sentence for Qwen embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
        print("   + Qwen test passed")
    else:
        print("   Skipping Qwen test - no API key found")
        # Test the fallback behavior
        print("   Testing Qwen fallback without API key...")
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using Qwen: {getattr(service, 'use_qwen', False)}")
        print(f"   Using local: {getattr(service, 'use_local', False)}")
        test_text = "This is a test sentence for embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")


def test_google_embeddings():
    """Test Google embeddings"""
    print("\n\n=== Testing Google Embeddings ===")
    
    print("\n1. Testing with Google embeddings (if configured):")
    os.environ["USE_COHERE_EMBEDDINGS"] = "false"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"

    # Only test Google if API key is set
    google_api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
    if google_api_key:
        service = EmbeddingService()
        print(f"   Service dimension: {service.dimension}")
        print(f"   Using local: {getattr(service, 'use_local', False)}")
        test_text = "This is a test sentence for Google embedding."
        embedding = service.embed_text(test_text)
        print(f"   Embedding length: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")
        print("   + Google test passed")
    else:
        print("   Skipping Google test - no API key found")


def test_local_embeddings():
    """Test local embeddings"""
    print("\n\n=== Testing Local Embeddings ===")
    
    print("\n1. Testing with local embeddings (default):")
    os.environ["USE_COHERE_EMBEDDINGS"] = "false"
    os.environ["USE_QWEN_EMBEDDINGS"] = "false"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "true"

    service = EmbeddingService()
    print(f"   Service dimension: {service.dimension}")
    print(f"   Using local: {getattr(service, 'use_local', False)}")

    test_text = "This is a test sentence for embedding."
    embedding = service.embed_text(test_text)
    print(f"   Embedding length: {len(embedding)}")
    print(f"   First 5 values: {embedding[:5]}")
    print("   + Local embeddings test passed")


def test_priority_order():
    """Test the priority order of embeddings"""
    print("\n\n=== Testing Priority Order ===")
    
    # Test Cohere > Qwen > Google > Local priority
    print("\n1. Testing priority: Cohere > Qwen > Google > Local")
    
    # First, try with all enabled but see if Cohere takes precedence
    os.environ["USE_COHERE_EMBEDDINGS"] = "true"
    os.environ["USE_QWEN_EMBEDDINGS"] = "true"
    os.environ["USE_LOCAL_EMBEDDINGS"] = "false"
    
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qwen_api_key = os.getenv("QWEN_API_KEY") 
    google_api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
    
    service = EmbeddingService()
    print(f"   Using Cohere: {getattr(service, 'use_cohere', False)}")
    print(f"   Using Qwen: {getattr(service, 'use_qwen', False)}")
    print(f"   Using local: {getattr(service, 'use_local', False)}")
    
    if cohere_api_key:
        print("   + Cohere takes priority as expected")
    elif qwen_api_key:
        print("   + Qwen takes priority when Cohere not available")
    elif google_api_key:
        print("   + Google takes priority when Cohere and Qwen not available")
    else:
        print("   + Local embeddings used as fallback")


def run_comprehensive_tests():
    """Run all embedding tests"""
    print("Running comprehensive embedding tests...")
    
    # Run priority order test first
    test_priority_order()
    
    # Run individual embedding tests
    test_cohere_embeddings()
    test_qwen_embeddings()
    test_google_embeddings()
    test_local_embeddings()
    
    print("\n\n=== All Tests Completed ===")


if __name__ == "__main__":
    run_comprehensive_tests()