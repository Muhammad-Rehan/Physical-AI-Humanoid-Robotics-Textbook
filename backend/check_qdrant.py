#!/usr/bin/env python3
"""
Check if Qdrant is running and accessible
"""
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv
load_dotenv()

def check_qdrant():
    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    
    print(f"Attempting to connect to Qdrant at: {qdrant_url}")
    
    try:
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=5
        )
        
        # Try to get collections to test connection
        collections = client.get_collections()
        print("+ Successfully connected to Qdrant!")
        print(f"Available collections: {[col.name for col in collections.collections]}")
        return client
    except Exception as e:
        print(f"- Could not connect to Qdrant: {e}")

        # Try default localhost
        try:
            print("Trying default localhost:6333...")
            client = QdrantClient(host="localhost", port=6333, timeout=5)
            collections = client.get_collections()
            print("+ Successfully connected to Qdrant at localhost:6333!")
            print(f"Available collections: {[col.name for col in collections.collections]}")
            return client
        except Exception as e2:
            print(f"- Could not connect to Qdrant at localhost:6333: {e2}")
            print("\nTo run Qdrant locally, you can use Docker:")
            print("   docker run -d --name qdrant -p 6333:6333 qdrant/qdrant")
            print("\nOr you can set up a cloud Qdrant instance and update your .env file")
            return None

if __name__ == "__main__":
    check_qdrant()