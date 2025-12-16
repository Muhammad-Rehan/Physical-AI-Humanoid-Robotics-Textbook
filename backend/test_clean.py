#!/usr/bin/env python3
"""
Test with clean environment variables to verify all paths work
"""
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Clean environment - remove all embedding-related env vars
for var in ['USE_QWEN_EMBEDDINGS', 'USE_LOCAL_EMBEDDINGS', 'GOOGLE_GEMINI_API_KEY', 'QWEN_API_KEY']:
    os.environ.pop(var, None)

print("=== Test 1: Default behavior (should use Google if API key available) ===")
# Set only Google API key
os.environ["GOOGLE_GEMINI_API_KEY"] = "fake_key_for_testing"

from src.services.embedding_service import EmbeddingService

service1 = EmbeddingService()
print(f"   Service dimension: {service1.dimension}")
print(f"   Has use_qwen: {hasattr(service1, 'use_qwen')}")
print(f"   Using Qwen: {getattr(service1, 'use_qwen', False)}")
print(f"   Using local: {getattr(service1, 'use_local', False)}")

print("\n=== Test 2: Qwen enabled with fake API key ===")
# Now test Qwen
os.environ["USE_QWEN_EMBEDDINGS"] = "true"
os.environ["QWEN_API_KEY"] = "fake_qwen_key"

# Since dashscope is not installed in this environment, it should fall back
service2 = EmbeddingService()
print(f"   Service dimension: {service2.dimension}")
print(f"   Has use_qwen: {hasattr(service2, 'use_qwen')}")
print(f"   Using Qwen: {getattr(service2, 'use_qwen', False)}")
print(f"   Using local: {getattr(service2, 'use_local', False)}")

print("\n=== Test 3: Qwen enabled but dashscope not available ===")
# Test the case where dashscope is not available
service3 = EmbeddingService()
test_text = "Test embedding"
embedding = service3.embed_text(test_text)
print(f"   Embedding length: {len(embedding)}")
print(f"   Using local after fallback: {getattr(service3, 'use_local', False)}")

print("\n=== Test 4: Local embeddings only ===")
# Reset and test local only
for var in ['USE_QWEN_EMBEDDINGS', 'USE_LOCAL_EMBEDDINGS', 'GOOGLE_GEMINI_API_KEY', 'QWEN_API_KEY']:
    os.environ.pop(var, None)

os.environ["USE_LOCAL_EMBEDDINGS"] = "true"
service4 = EmbeddingService()
print(f"   Service dimension: {service4.dimension}")
print(f"   Using local: {getattr(service4, 'use_local', False)}")

print("\nAll tests completed successfully!")