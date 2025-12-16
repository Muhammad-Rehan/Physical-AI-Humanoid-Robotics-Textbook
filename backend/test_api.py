import requests
import json

# Test the RAG chatbot API
def test_api():
    # Test URL - adjust if your API is running on a different port
    base_url = "http://localhost:8000"

    # Test the health endpoint
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{base_url}/")
        print(f"Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")

    # Test the chat endpoint
    print("\nTesting chat endpoint...")
    try:
        # Test question
        test_payload = {
            "question": "What is Physical AI?",
            "selected_text": None
        }

        response = requests.post(
            f"{base_url}/api/v1/chat/ask",
            headers={"Content-Type": "application/json"},
            data=json.dumps(test_payload)
        )

        print(f"Chat endpoint: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Answer: {result.get('answer', 'No answer field')[:100]}...")
            print(f"Sources: {result.get('sources', [])}")
        else:
            print(f"Error response: {response.text}")
    except Exception as e:
        print(f"Chat endpoint test failed: {e}")

    # Test selected text mode
    print("\nTesting selected text mode...")
    try:
        test_payload = {
            "question": "Explain this concept?",
            "selected_text": "Physical AI is a field that combines principles of physics with artificial intelligence to create more robust and efficient AI systems."
        }

        response = requests.post(
            f"{base_url}/api/v1/chat/ask",
            headers={"Content-Type": "application/json"},
            data=json.dumps(test_payload)
        )

        print(f"Selected text mode: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Answer: {result.get('answer', 'No answer field')[:100]}...")
            print(f"Selected text mode: {result.get('selected_text_mode', False)}")
        else:
            print(f"Error response: {response.text}")
    except Exception as e:
        print(f"Selected text mode test failed: {e}")

if __name__ == "__main__":
    print("Testing RAG Chatbot API...")
    test_api()
    print("\nTesting complete!")