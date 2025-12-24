#!/usr/bin/env python3
"""
Script to ingest book data from website into Qdrant using Cohere embeddings
"""
import os
import asyncio
import re
from typing import List, Dict, Any
from urllib.parse import urljoin, urlparse
import requests
from bs4 import BeautifulSoup
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

class BookDataIngestor:
    def __init__(self):
        # Initialize Cohere client
        cohere_api_key = os.getenv("COHERE_API_KEY")
        if not cohere_api_key:
            raise ValueError("COHERE_API_KEY not found in environment variables")
        self.cohere_client = cohere.Client(cohere_api_key)
        
        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=10
        )
        
        # Collection name for book data
        self.collection_name = "book_content"
        
        # Base URL for the book
        self.base_url = "https://muhammad-rehan.github.io/Physical-AI-Humanoid-Robotics-Textbook"
        
        # Pages to exclude from ingestion
        self.exclude_patterns = [
            "/blog/",
            "/authors/",
            "/tags/",
            "/archive/",
            "/tutorial-",
            "/markdown-page"
        ]

    def extract_content_from_page(self, url: str) -> Dict[str, Any]:
        """Extract main content from a page"""
        try:
            response = requests.get(url)
            response.raise_for_status()
            
            soup = BeautifulSoup(response.content, 'html.parser')
            
            # Remove navigation, headers, footers, and other non-content elements
            for element in soup(['nav', 'header', 'footer', 'aside', 'script', 'style']):
                element.decompose()
            
            # Try to find main content container (common selectors for Docusaurus sites)
            main_content = (
                soup.find('main') or
                soup.find('article') or
                soup.find('div', class_='main-wrapper') or
                soup.find('div', {'role': 'main'}) or
                soup.find('div', class_=re.compile(r'.*docItemContainer.*')) or
                soup
            )
            
            # Extract text content
            text_content = main_content.get_text(separator=' ', strip=True)
            
            # Remove excessive whitespace
            text_content = re.sub(r'\s+', ' ', text_content)
            
            # Get title
            title = ""
            title_elem = soup.find('title')
            if title_elem:
                title = title_elem.get_text().strip()
            else:
                h1_elem = soup.find('h1')
                if h1_elem:
                    title = h1_elem.get_text().strip()
            
            return {
                "url": url,
                "title": title,
                "content": text_content,
                "html": str(main_content)
            }
        except Exception as e:
            print(f"Error extracting content from {url}: {e}")
            return None

    def chunk_text(self, text: str, max_chunk_size: int = 1000, overlap: int = 100) -> List[str]:
        """Split text into overlapping chunks for better embedding"""
        if len(text) <= max_chunk_size:
            return [text]
        
        chunks = []
        start = 0
        
        while start < len(text):
            end = start + max_chunk_size
            
            # If we're not at the end, try to break at sentence boundary
            if end < len(text):
                # Look for a sentence boundary near the end
                sentence_end = text.rfind('.', start, end)
                if sentence_end > start + 500:  # Only if it's reasonably close to the chunk size
                    end = sentence_end + 1
                else:
                    # Look for a paragraph break
                    para_end = text.rfind('\n\n', start, end)
                    if para_end > start + 500:
                        end = para_end + 2
                    else:
                        # Look for a space break
                        space_end = text.rfind(' ', start, end)
                        if space_end > start + 500:
                            end = space_end
            
            chunk = text[start:end].strip()
            if chunk:
                chunks.append(chunk)
            
            start = end - overlap if end < len(text) else len(text)
        
        return chunks

    def get_sitemap_urls(self, sitemap_url: str) -> List[str]:
        """Extract all URLs from sitemap"""
        try:
            response = requests.get(sitemap_url)
            response.raise_for_status()

            # Try XML parser with lxml, fallback to html parser
            try:
                soup = BeautifulSoup(response.content, 'xml')
            except:
                # If xml parser fails, try html parser
                soup = BeautifulSoup(response.content, 'html.parser')

            urls = []
            # Look for both XML and HTML sitemap formats
            for loc in soup.find_all(['loc', 'url']):
                url = loc.text.strip()
                if url:  # Check if URL is not empty
                    # Skip excluded patterns
                    should_exclude = any(pattern in url for pattern in self.exclude_patterns)
                    if not should_exclude:
                        urls.append(url)

            # Filter out non-HTTP URLs and ensure they're from the correct domain
            valid_urls = []
            for url in urls:
                if url.startswith('http') and 'Physical-AI-Humanoid-Robotics-Textbook' in url:
                    valid_urls.append(url)

            print(f"Found {len(valid_urls)} valid URLs in sitemap")
            return valid_urls
        except Exception as e:
            print(f"Error fetching sitemap: {e}")
            return []

    def create_collection(self):
        """Create Qdrant collection with appropriate vector size for Cohere embeddings"""
        # Cohere embeddings have dimension of 1024 for multilingual-v3.0
        vector_size = 1024

        try:
            # Check if collection exists
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if collection_exists:
                print(f"Collection '{self.collection_name}' already exists")
                return

            # Create collection
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )

            print(f"Created collection '{self.collection_name}' with {vector_size}-dimension vectors")
        except Exception as e:
            print(f"Error creating collection: {e}")
            # If Qdrant is not running locally, try to connect to a default instance
            print("Trying to connect to default Qdrant instance...")
            try:
                # Try to create client with default settings
                temp_client = QdrantClient(host="localhost", port=6333)
                collections = temp_client.get_collections()
                collection_exists = any(col.name == self.collection_name for col in collections.collections)

                if not collection_exists:
                    temp_client.create_collection(
                        collection_name=self.collection_name,
                        vectors_config=models.VectorParams(
                            size=vector_size,
                            distance=models.Distance.COSINE
                        )
                    )
                    print(f"Created collection with default Qdrant settings")
                else:
                    print(f"Collection already exists with default settings")
            except Exception as e2:
                print(f"Could not connect to Qdrant: {e2}")
                print("Please make sure Qdrant is running before running this script")
                raise e2

    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """Generate Cohere embeddings for a list of texts"""
        try:
            response = self.cohere_client.embed(
                texts=texts,
                model=os.getenv("COHERE_EMBED_MODEL", "embed-multilingual-v3.0"),
                input_type=input_type
            )
            return response.embeddings
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            return None

    def ingest_page_content(self, url: str):
        """Ingest a single page into Qdrant"""
        print(f"Processing: {url}")
        
        content_data = self.extract_content_from_page(url)
        if not content_data:
            return
        
        # Chunk the content
        content_chunks = self.chunk_text(content_data["content"])
        print(f"  - Split into {len(content_chunks)} content chunks")
        
        # Generate embeddings for all chunks
        embeddings = self.generate_embeddings(content_chunks)
        if embeddings is None:
            print(f"  - Failed to generate embeddings for {url}")
            return
        
        # Prepare points for Qdrant
        points = []
        for i, (chunk, embedding) in enumerate(zip(content_chunks, embeddings)):
            point = models.PointStruct(
                id=f"{url}#{i}",
                vector=embedding,
                payload={
                    "url": url,
                    "title": content_data["title"],
                    "content": chunk,
                    "chunk_index": i,
                    "total_chunks": len(content_chunks)
                }
            )
            points.append(point)
        
        # Upload to Qdrant
        try:
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            print(f"  - Uploaded {len(points)} chunks to Qdrant")
        except Exception as e:
            print(f"  - Error uploading points to Qdrant: {e}")

    def ingest_book_data(self, sitemap_url: str):
        """Main method to ingest all book data"""
        print("Starting book data ingestion process...")
        
        # Create collection if it doesn't exist
        self.create_collection()
        
        # Get all URLs from sitemap
        urls = self.get_sitemap_urls(sitemap_url)
        
        # Process each URL
        for i, url in enumerate(urls):
            print(f"\nProcessing {i+1}/{len(urls)}: {url}")
            self.ingest_page_content(url)
        
        print(f"\nIngestion complete! Processed {len(urls)} pages.")

def main():
    # Initialize the ingester
    ingester = BookDataIngestor()
    
    # Sitemap URL
    sitemap_url = "https://muhammad-rehan.github.io/Physical-AI-Humanoid-Robotics-Textbook/sitemap.xml"
    
    # Start ingestion
    ingester.ingest_book_data(sitemap_url)

if __name__ == "__main__":
    main()