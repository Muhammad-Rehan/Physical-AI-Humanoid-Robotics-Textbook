#!/usr/bin/env python3
"""
Script to load processed book data into Qdrant using Cohere embeddings
This script should be run after Qdrant is set up and running.
"""
import os
import json
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from dotenv import load_dotenv

load_dotenv()

def load_data_to_qdrant():
    """Load processed book data into Qdrant"""
    
    # Initialize Cohere client
    cohere_api_key = os.getenv("COHERE_API_KEY")
    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY not found in environment variables")
    cohere_client = cohere.Client(cohere_api_key)
    
    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    
    print(f"Connecting to Qdrant at {qdrant_url}")
    try:
        qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=10
        )
    except Exception as e:
        print(f"Could not connect to Qdrant: {e}")
        print("Please make sure Qdrant is running before executing this script.")
        print("You can run Qdrant with Docker using: docker run -d --name qdrant -p 6333:6333 qdrant/qdrant")
        return
    
    # Collection name for book data
    collection_name = "book_content"
    
    # Create collection with appropriate vector size for Cohere embeddings
    vector_size = 1024  # Cohere multilingual-v3.0 embeddings are 1024-dimensional
    
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)
        
        if collection_exists:
            print(f"Collection '{collection_name}' already exists")
            # Get current points count
            count = qdrant_client.count(collection_name=collection_name)
            print(f"Current points in collection: {count.count}")
        else:
            # Create collection
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=vector_size,
                    distance=models.Distance.COSINE
                )
            )
            print(f"Created collection '{collection_name}' with {vector_size}-dimension vectors")
    except Exception as e:
        print(f"Error with collection: {e}")
        return
    
    # Load processed data
    try:
        with open('processed_book_data.json', 'r', encoding='utf-8') as f:
            data = json.load(f)
        print(f"Loaded {len(data)} processed content chunks")
    except FileNotFoundError:
        print("processed_book_data.json not found. Please run process_book_data.py first.")
        return
    except Exception as e:
        print(f"Error loading processed data: {e}")
        return
    
    # Prepare points for Qdrant
    points = []
    for idx, item in enumerate(data):
        # If embedding is not already generated, generate it
        embedding = item.get('embedding')
        if embedding is None:
            print(f"Generating embedding for {item['url']}")
            try:
                response = cohere_client.embed(
                    texts=[item['content']],
                    model=os.getenv("COHERE_EMBED_MODEL", "embed-multilingual-v3.0"),
                    input_type="search_document"
                )
                embedding = response.embeddings[0]
            except Exception as e:
                print(f"Error generating embedding: {e}")
                continue  # Skip this item if we can't generate embedding

        point = models.PointStruct(
            id=idx,  # Use numeric ID instead of URL-based ID
            vector=embedding,
            payload={
                "url": item['url'],
                "title": item['title'],
                "content": item['content'],
                "chunk_index": item['chunk_index'],
                "total_chunks": item['total_chunks']
            }
        )
        points.append(point)
    
    print(f"Preparing to upload {len(points)} vectors to Qdrant...")
    
    # Upload to Qdrant in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        try:
            qdrant_client.upsert(
                collection_name=collection_name,
                points=batch
            )
            print(f"Uploaded batch {i//batch_size + 1}/{(len(points)-1)//batch_size + 1}")
        except Exception as e:
            print(f"Error uploading batch {i//batch_size + 1}: {e}")
            continue
    
    print(f"Successfully loaded book data into Qdrant collection '{collection_name}'")
    
    # Verify the upload
    count = qdrant_client.count(collection_name=collection_name)
    print(f"Total points in collection after upload: {count.count}")

def main():
    print("Loading processed book data to Qdrant...")
    print("Make sure Qdrant is running before proceeding.")
    
    load_data_to_qdrant()

if __name__ == "__main__":
    main()