#!/usr/bin/env python3
"""
Script to download and prepare book data from website, 
with option to store in Qdrant when available
"""
import os
import asyncio
import re
from typing import List, Dict, Any
from urllib.parse import urljoin, urlparse
import requests
from bs4 import BeautifulSoup
import cohere
import json

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

    def save_processed_data(self, data: List[Dict], filename: str = "processed_book_data.json"):
        """Save processed data to a JSON file"""
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        print(f"Saved {len(data)} processed items to {filename}")

    def process_book_data(self, sitemap_url: str):
        """Main method to process all book data - without Qdrant"""
        print("Starting book data processing (without Qdrant)...")
        
        # Get all URLs from sitemap
        urls = self.get_sitemap_urls(sitemap_url)
        
        processed_data = []
        
        # Process each URL
        for i, url in enumerate(urls):
            print(f"\nProcessing {i+1}/{len(urls)}: {url}")
            
            content_data = self.extract_content_from_page(url)
            if not content_data:
                continue
            
            # Chunk the content
            content_chunks = self.chunk_text(content_data["content"])
            print(f"  - Split into {len(content_chunks)} content chunks")
            
            # Generate embeddings for all chunks
            embeddings = self.generate_embeddings(content_chunks)
            if embeddings is None:
                print(f"  - Failed to generate embeddings for {url}")
                # Still add the data without embeddings
                for j, chunk in enumerate(content_chunks):
                    processed_data.append({
                        "id": f"{url}#{j}",
                        "url": url,
                        "title": content_data["title"],
                        "content": chunk,
                        "chunk_index": j,
                        "total_chunks": len(content_chunks),
                        "embedding": None  # Will be added when Qdrant is available
                    })
            else:
                # Add content with embeddings
                for j, (chunk, embedding) in enumerate(zip(content_chunks, embeddings)):
                    processed_data.append({
                        "id": f"{url}#{j}",
                        "url": url,
                        "title": content_data["title"],
                        "content": chunk,
                        "chunk_index": j,
                        "total_chunks": len(content_chunks),
                        "embedding": embedding
                    })
        
        # Save processed data to file
        self.save_processed_data(processed_data)
        
        print(f"\nProcessing complete! Processed {len(urls)} pages.")
        print(f"Total chunks: {len(processed_data)}")
        return processed_data

def main():
    # Initialize the ingester
    ingester = BookDataIngestor()
    
    # Sitemap URL
    sitemap_url = "https://muhammad-rehan.github.io/Physical-AI-Humanoid-Robotics-Textbook/sitemap.xml"
    
    # Start processing
    processed_data = ingester.process_book_data(sitemap_url)

if __name__ == "__main__":
    main()