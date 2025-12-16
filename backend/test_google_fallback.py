#!/usr/bin/env python3
"""
Test script to verify Google embedding fallback still works properly
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Set up a fake Google API key to test the Google embedding path
os.environ["GOOGLE_GEMINI_API_KEY"] = "fake_key_for_testing"

from src.services.embedding_service import EmbeddingService

def test_google_fallback():
    print("Testing Google embedding path (with fake API key to trigger the API call attempt)...")

    service = EmbeddingService()
    print(f"   Service dimension: {service.dimension}")
    print(f"   Using local: {getattr(service, 'use_local', False)}")

    # This should try to call Google's API but fail due to the fake key
    # and then should fallback to local embeddings
    test_text = "This is a test sentence for embedding."
    embedding = service.embed_text(test_text)
    print(f"   Embedding length: {len(embedding)}")
    print(f"   First 5 values: {embedding[:5]}")
    print(f"   Final state - Using local: {getattr(service, 'use_local', False)}")

if __name__ == "__main__":
    test_google_fallback()