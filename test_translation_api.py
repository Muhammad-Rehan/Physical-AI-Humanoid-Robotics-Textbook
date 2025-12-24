#!/usr/bin/env python3
"""
Simple test script to verify the translation API is working correctly
"""
import requests
import json

def test_translation_api():
    """Test the translation API endpoint"""
    # Test URL - adjust this to match your backend URL
    base_url = "http://localhost:8000/api/v1"

    # Test translation endpoint
    translation_url = f"{base_url}/translate"

    # Sample translation request
    payload = {
        "text": "Hello, world! This is a test translation.",
        "target_language": "es",  # Spanish
        "source_language": "en",  # English
        "context": "page_content"
    }

    try:
        print("Testing translation API...")
        response = requests.post(translation_url, json=payload)

        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.text}")

        if response.status_code == 200:
            result = response.json()
            print(f"Translated text: {result.get('translated_text', 'N/A')}")
            print("Translation API is working correctly!")
        else:
            print("Translation API returned an error.")

    except Exception as e:
        print(f"Error testing translation API: {e}")
        print("Make sure the backend server is running on http://localhost:8000")

def test_health_api():
    """Test the health check endpoint"""
    base_url = "http://localhost:8000/api/v1"
    health_url = f"{base_url}/health"

    try:
        print("\nTesting health API...")
        response = requests.get(health_url)

        print(f"Health Status Code: {response.status_code}")
        print(f"Health Response: {response.text}")

    except Exception as e:
        print(f"Error testing health API: {e}")

if __name__ == "__main__":
    print("Testing Translation API...")
    test_health_api()  # Test health first
    test_translation_api()