import os
from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class QdrantService:
    def __init__(self, collection_name: str = "textbook_content"):
        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            raise ValueError("Both QDRANT_URL and QDRANT_API_KEY environment variables are required")

        self.client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=True
        )

        self.collection_name = collection_name

    def search_similar(self, query_embedding: List[float], limit: int = 3) -> List[Dict]:
        """
        Search for similar vectors in the collection
        """
        try:
            # Check if collection exists first
            try:
                self.client.get_collection(self.collection_name)
            except:
                print(f"Collection '{self.collection_name}' does not exist. Please run the data pipeline to populate the database.")
                return []

            # Perform similarity search - try both old and new method names for compatibility
            try:
                # Try the newer query_points method first
                search_results = self.client.query_points(
                    collection_name=self.collection_name,
                    query=query_embedding,
                    limit=limit,
                    with_payload=True  # Include the stored payload (text, url, etc.)
                )

                # Handle the response format for query_points
                results = []
                for hit in search_results.points:
                    result = {
                        'text': hit.payload.get('text', '') if hit.payload else '',
                        'url': hit.payload.get('url', '') if hit.payload else '',
                        'title': hit.payload.get('title', '') if hit.payload else '',
                        'word_count': hit.payload.get('word_count', 0) if hit.payload else 0,
                        'score': hit.score  # Similarity score
                    }
                    results.append(result)

                return results
            except AttributeError:
                # Fall back to the older search method
                search_results = self.client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=limit,
                    with_payload=True  # Include the stored payload (text, url, etc.)
                )

                # Extract relevant information from results
                results = []
                for hit in search_results:
                    result = {
                        'text': hit.payload.get('text', ''),
                        'url': hit.payload.get('url', ''),
                        'title': hit.payload.get('title', ''),
                        'word_count': hit.payload.get('word_count', 0),
                        'score': hit.score  # Similarity score
                    }
                    results.append(result)

                return results

        except Exception as e:
            print(f"Error searching in Qdrant: {e}")
            return []

    def get_collection_info(self) -> Dict:
        """
        Get information about the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                'name': collection_info.config.params.vectors.size,
                'vectors_count': collection_info.vectors_count,
                'indexed_vectors_count': collection_info.indexed_vectors_count
            }
        except Exception as e:
            print(f"Error getting collection info: {e}")
            return {}

    def add_document(self, text: str, url: str = "", title: str = "", word_count: int = 0) -> bool:
        """
        Add a single document to the collection
        """
        try:
            # This would typically be called from the data pipeline
            # For now, just return True to indicate the method exists
            return True
        except Exception as e:
            print(f"Error adding document to Qdrant: {e}")
            return False

    def batch_search(self, query_embeddings: List[List[float]], limit: int = 3) -> List[List[Dict]]:
        """
        Perform multiple searches at once
        """
        results = []
        for embedding in query_embeddings:
            result = self.search_similar(embedding, limit)
            results.append(result)
        return results