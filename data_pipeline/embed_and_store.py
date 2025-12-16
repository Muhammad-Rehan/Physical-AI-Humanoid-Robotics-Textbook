import os
import uuid
from typing import List, Dict
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import sys
import importlib.util

# Load environment variables from the parent directory
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'backend', '.env'))

# Add the backend path to sys.path to import the embedding service
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__)), 'backend', 'src'))

# Import the embedding service from the backend
from services.embedding_service import EmbeddingService as BackendEmbeddingService

class EmbeddingService:
    def __init__(self):
        # Use the backend's embedding service which supports both Google and local embeddings
        self.backend_embedding_service = BackendEmbeddingService()

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            raise ValueError("Both QDRANT_URL and QDRANT_API_KEY environment variables are required")

        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=True
        )

        # Model for embeddings - use 768 dimensions to match Google's embedding-001
        # The backend service will handle dimension appropriately

    def create_collection(self, collection_name: str = "textbook_content"):
        """
        Creates a Qdrant collection for storing text embeddings
        """
        try:
            # Check if collection already exists
            self.qdrant_client.get_collection(collection_name)
            print(f"Collection '{collection_name}' already exists")
        except:
            # Create new collection
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # Size of Google's embedding-001 vectors
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created new collection '{collection_name}'")

    def embed_text(self, text: str) -> List[float]:
        """
        Generates embedding for a single text using the backend service
        """
        try:
            # Use the backend's embedding service which handles both Google and local embeddings
            return self.backend_embedding_service.embed_text(text)
        except Exception as e:
            print(f"Error embedding text: {e}")
            # Return zero vector with appropriate dimension
            return [0.0] * 768  # Use 768 to match Google's embedding-001

    def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a batch of texts
        """
        embeddings = []
        for text in texts:
            embedding = self.embed_text(text)
            if embedding:
                embeddings.append(embedding)
            else:
                # Add a zero vector if embedding failed
                embeddings.append([0.0] * 768)
        return embeddings

    def store_chunks(self, chunks: List[Dict], collection_name: str = "textbook_content"):
        """
        Stores text chunks with their embeddings in Qdrant
        """
        # Prepare points for insertion
        points = []
        for chunk in chunks:
            # Generate embedding for the text
            embedding = self.embed_text(chunk['text'])

            if not embedding:
                print(f"Failed to generate embedding for chunk: {chunk.get('text', '')[:100]}...")
                continue

            # Create a unique ID for this chunk
            chunk_id = str(uuid.uuid4())

            # Create payload with chunk information
            payload = {
                "text": chunk['text'],
                "url": chunk.get('url', ''),
                "title": chunk.get('title', ''),
                "word_count": chunk.get('word_count', len(chunk['text'].split()))
            }

            # Add to points list
            points.append(
                models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload=payload
                )
            )

        # Upload points to Qdrant
        if points:
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=points
            )
            print(f"Stored {len(points)} chunks in Qdrant collection '{collection_name}'")
        else:
            print("No points to store")

    def process_and_store(self, scraped_content: List[Dict], collection_name: str = "textbook_content"):
        """
        Processes scraped content by chunking and storing in Qdrant
        """
        from chunk_text import chunk_by_paragraphs

        all_chunks = []

        for item in scraped_content:
            # Chunk the content
            chunks = chunk_by_paragraphs(item['content'])

            # Add source information to each chunk
            for chunk in chunks:
                chunk['url'] = item['url']
                chunk['title'] = item['title']
                all_chunks.append(chunk)

        print(f"Processing {len(all_chunks)} total chunks for storage")

        # Create collection if it doesn't exist
        self.create_collection(collection_name)

        # Store chunks in batches to avoid memory issues
        batch_size = 10
        for i in range(0, len(all_chunks), batch_size):
            batch = all_chunks[i:i + batch_size]
            self.store_chunks(batch, collection_name)

            print(f"Processed batch {i//batch_size + 1}/{(len(all_chunks)-1)//batch_size + 1}")

def load_scraped_content(file_path: str = "scraped_content.txt") -> List[Dict]:
    """
    Loads scraped content from file
    """
    content_items = []

    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Split by separator
        items = content.split("-" * 80)

        for item_text in items:
            if "URL:" in item_text and "CONTENT:" in item_text:
                lines = item_text.strip().split('\n')

                url = ""
                title = ""
                content_text = ""

                for line in lines:
                    if line.startswith("URL:"):
                        url = line[4:].strip()
                    elif line.startswith("TITLE:"):
                        title = line[6:].strip()
                    elif line.startswith("CONTENT:"):
                        content_text = line[8:].strip()

                if content_text:
                    content_items.append({
                        'url': url,
                        'title': title,
                        'content': content_text
                    })
    except FileNotFoundError:
        print(f"File {file_path} not found. Please run scrape_content.py first.")
        return []

    return content_items

if __name__ == "__main__":
    # Initialize the embedding service
    embedding_service = EmbeddingService()

    # Load scraped content
    scraped_content = load_scraped_content()

    if not scraped_content:
        print("No scraped content found. Please run scrape_content.py first.")
    else:
        print(f"Found {len(scraped_content)} scraped content items")

        # Process and store in Qdrant
        embedding_service.process_and_store(scraped_content)

        print("Embedding and storage process completed!")