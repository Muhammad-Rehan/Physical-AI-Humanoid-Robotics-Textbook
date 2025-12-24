from src.services.qdrant_service import QdrantService

# Create a service instance with the correct collection name
service = QdrantService(collection_name='book_content')

try:
    info = service.get_collection_info()
    print(f'Collection info: {info}')
except Exception as e:
    print(f"Error: {e}")
    
    # Try to search for a simple test
    try:
        # Create a simple embedding to test search
        from src.services.embedding_service import EmbeddingService
        emb_service = EmbeddingService()
        test_embedding = emb_service.embed_text("test")
        results = service.search_similar(test_embedding, limit=1)
        print(f'Number of results from search: {len(results)}')
        if results:
            print(f'First result: {results[0]}')
    except Exception as e2:
        print(f"Search also failed: {e2}")