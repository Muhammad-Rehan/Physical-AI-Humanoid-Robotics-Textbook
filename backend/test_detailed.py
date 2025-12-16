#!/usr/bin/env python3
"""
Detailed test to understand the embedding service initialization
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Clear environment variables for a clean test
os.environ.pop("USE_QWEN_EMBEDDINGS", None)
os.environ.pop("USE_LOCAL_EMBEDDINGS", None)

# Set up a fake Google API key
os.environ["GOOGLE_GEMINI_API_KEY"] = "fake_key_for_testing"

from src.services.embedding_service import EmbeddingService

def test_detailed():
    print("Testing EmbeddingService initialization with Google API key...")

    # Check what the environment variables are set to
    print(f"USE_QWEN_EMBEDDINGS: {os.getenv('USE_QWEN_EMBEDDINGS', 'not set')}")
    print(f"USE_LOCAL_EMBEDDINGS: {os.getenv('USE_LOCAL_EMBEDDINGS', 'not set')}")
    print(f"GOOGLE_GEMINI_API_KEY: {'set' if os.getenv('GOOGLE_GEMINI_API_KEY') else 'not set'}")

    service = EmbeddingService()
    print(f"   Service dimension: {service.dimension}")
    print(f"   Using local: {getattr(service, 'use_local', False)}")
    print(f"   Has use_qwen attribute: {hasattr(service, 'use_qwen')}")
    if hasattr(service, 'use_qwen'):
        print(f"   Using Qwen: {service.use_qwen}")

    # Test with Qwen enabled but no API key
    print("\nTesting with Qwen enabled but no API key...")
    os.environ["USE_QWEN_EMBEDDINGS"] = "true"
    os.environ.pop("QWEN_API_KEY", None)  # Make sure no Qwen API key is set

    service2 = EmbeddingService()
    print(f"   Service dimension: {service2.dimension}")
    print(f"   Has use_qwen attribute: {hasattr(service2, 'use_qwen')}")
    if hasattr(service2, 'use_qwen'):
        print(f"   Using Qwen: {service2.use_qwen}")
    print(f"   Using local: {getattr(service2, 'use_local', False)}")

if __name__ == "__main__":
    test_detailed()